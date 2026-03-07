package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.FlywheelConstants;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {

    // Hardware
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    // Control requests
    private final VelocityTorqueCurrentFOC velocityRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<AngularVelocity> leaderVelocity;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // State
    private double targetVelocityRPM = 0.0;

    public Flywheel() {
        // Initialize motors
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        leaderMotor = new TalonFX(CANIds.FLYWHEEL_LEADER_MOTOR, canBus);
        followerMotor = new TalonFX(CANIds.FLYWHEEL_FOLLOWER_MOTOR, canBus);

        // Initialize control requests
        velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure motors
        configureLeaderMotor();
        configureFollowerMotor();

        // Cache status signals
        leaderVelocity = leaderMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                leaderVelocity,
                leaderMotor.getPosition(),
                leaderMotor.getMotorVoltage());

        // Optimize CAN bus utilization
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();

        // Configure SysId routine for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Start with 4V
                        null, // Use default timeout (10 s)
                        (state) -> SignalLogger.writeString("flywheel-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> leaderMotor.setControl(voltageRequest.withOutput(voltage.in(Volts))),
                        null,
                        this));
    }

    private void configureLeaderMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;

        config.Slot0.kS = FlywheelConstants.S;
        config.Slot0.kV = FlywheelConstants.V;
        config.Slot0.kA = FlywheelConstants.A;
        config.Slot0.kP = FlywheelConstants.P;
        config.Slot0.kI = FlywheelConstants.I;
        config.Slot0.kD = FlywheelConstants.D;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.SUPPLY_CURRENT_LOWER_TIME;

        config.TorqueCurrent.PeakForwardTorqueCurrent = FlywheelConstants.STATOR_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -FlywheelConstants.STATOR_CURRENT_LIMIT;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = leaderMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure flywheel leader motor: " + status);
        }
    }

    private void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.SUPPLY_CURRENT_LOWER_TIME;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = followerMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure flywheel follower motor: " + status);
        }

        followerMotor.setControl(new Follower(CANIds.FLYWHEEL_LEADER_MOTOR, MotorAlignmentValue.Opposed));
    }

    public void setVelocity(double rpm) {
        targetVelocityRPM = Math.max(0, rpm);
        if (targetVelocityRPM == 0.0) {
            leaderMotor.setControl(neutralRequest);
        } else {
            double rps = targetVelocityRPM / 60.0;
            leaderMotor.setControl(velocityRequest.withVelocity(rps));
        }
    }

    public boolean atSetpoint() {
        double currentRPM = getCurrentVelocityRPM();
        if (targetVelocityRPM == 0.0) {
            return Math.abs(currentRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
        }
        return Math.abs(currentRPM - targetVelocityRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public double getCurrentVelocityRPM() {
        BaseStatusSignal.refreshAll(leaderVelocity);
        return leaderVelocity.getValue().in(RotationsPerSecond) * 60.0;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Flywheel/Velocity RPM", getCurrentVelocityRPM());
        // SmartDashboard.putNumber("Flywheel/Target RPM", targetVelocityRPM);
        // SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}
