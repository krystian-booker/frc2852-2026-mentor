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

import frc.robot.Constants.FlywheelConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Flywheel subsystem using dual Kraken X60 motors with Phoenix Pro FOC control.
 */
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
        CANBus canBus = new CANBus(FlywheelConstants.kCANBus);
        leaderMotor = new TalonFX(FlywheelConstants.kLeaderMotorId, canBus);
        followerMotor = new TalonFX(FlywheelConstants.kFollowerMotorId, canBus);

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
                FlywheelConstants.kSignalUpdateFrequencyHz,
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = FlywheelConstants.kGearRatio;

        config.Slot0.kS = FlywheelConstants.kS;
        config.Slot0.kV = FlywheelConstants.kV;
        config.Slot0.kA = FlywheelConstants.kA;
        config.Slot0.kP = FlywheelConstants.kP;
        config.Slot0.kI = FlywheelConstants.kI;
        config.Slot0.kD = FlywheelConstants.kD;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.kSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.kSupplyCurrentLowerTime;

        config.TorqueCurrent.PeakForwardTorqueCurrent = FlywheelConstants.kStatorCurrentLimit;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -FlywheelConstants.kStatorCurrentLimit;

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
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = FlywheelConstants.kSupplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = FlywheelConstants.kSupplyCurrentLowerTime;

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

        followerMotor.setControl(new Follower(FlywheelConstants.kLeaderMotorId, MotorAlignmentValue.Opposed));
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
            return Math.abs(currentRPM) < FlywheelConstants.kVelocityToleranceRPM;
        }
        return Math.abs(currentRPM - targetVelocityRPM) < FlywheelConstants.kVelocityToleranceRPM;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private double getCurrentVelocityRPM() {
        BaseStatusSignal.refreshAll(leaderVelocity);
        return leaderVelocity.getValue().in(RotationsPerSecond) * 60.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/Velocity RPM", getCurrentVelocityRPM());
        SmartDashboard.putNumber("Flywheel/Target RPM", targetVelocityRPM);
        SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}