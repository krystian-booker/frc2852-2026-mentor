package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private final TorqueCurrentFOC torqueCurrentRequest;
    private final VoltageOut voltageRequest;
    private final NeutralOut neutralRequest;

    // Status signals
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Voltage> supplyVoltage;

    // State
    private double targetVelocityRPM = 0.0;

    public Flywheel() {
        // Initialize motors
        leaderMotor = new TalonFX(CANIds.FLYWHEEL_LEADER_MOTOR, TunerConstants.kCANBus);
        followerMotor = new TalonFX(CANIds.FLYWHEEL_FOLLOWER_MOTOR, TunerConstants.kCANBus);

        // Initialize control requests
        velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        torqueCurrentRequest = new TorqueCurrentFOC(0);
        voltageRequest = new VoltageOut(0);
        neutralRequest = new NeutralOut();

        // Configure motors
        configureLeaderMotor();
        configureFollowerMotor();

        // Cache status signals
        leaderVelocity = leaderMotor.getVelocity();
        statorCurrent = leaderMotor.getStatorCurrent();
        supplyVoltage = leaderMotor.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                leaderVelocity,
                statorCurrent,
                supplyVoltage,
                leaderMotor.getPosition(),
                leaderMotor.getMotorVoltage());

        // Optimize CAN bus utilization
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
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

    public void setBangBangVelocity(double rpm) {
        targetVelocityRPM = Math.max(0, rpm);
        if (targetVelocityRPM == 0.0) {
            leaderMotor.setControl(neutralRequest);
        } else {
            if (getCurrentVelocityRPM() < targetVelocityRPM) {
                leaderMotor.setControl(voltageRequest.withOutput(12.0));
            } else {
                leaderMotor.setControl(voltageRequest.withOutput(0.0));
            }
        }
    }

    public boolean atSetpoint() {
        double currentRPM = getCurrentVelocityRPM();
        if (targetVelocityRPM == 0.0) {
            return Math.abs(currentRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
        }
        return Math.abs(currentRPM - targetVelocityRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
    }

    public double getCurrentVelocityRPM() {
        BaseStatusSignal.refreshAll(leaderVelocity);
        return leaderVelocity.getValue().in(RotationsPerSecond) * 60.0;
    }

    /** Get mechanism velocity in rotations per second. */
    public double getVelocityRPS() {
        BaseStatusSignal.refreshAll(leaderVelocity);
        return leaderVelocity.getValue().in(RotationsPerSecond);
    }

    /** Get leader motor stator current in Amps. */
    public double getStatorCurrent() {
        BaseStatusSignal.refreshAll(statorCurrent);
        return statorCurrent.getValue().in(Amps);
    }

    /** Apply raw torque current output for feedforward characterization. */
    public void applyCurrent(double amps) {
        leaderMotor.setControl(torqueCurrentRequest.withOutput(amps));
    }

    /** Apply raw voltage output for system identification characterization. */
    public void applyVoltage(double volts) {
        leaderMotor.setControl(voltageRequest.withOutput(volts));
    }

    /** Get applied motor voltage in Volts. */
    public double getMotorVoltage() {
        return leaderMotor.getMotorVoltage().refresh().getValue().in(Volts);
    }

    /** Get supply (battery) voltage in Volts. */
    public double getSupplyVoltage() {
        BaseStatusSignal.refreshAll(supplyVoltage);
        return supplyVoltage.getValue().in(Volts);
    }

    /** Stop the flywheel and clear target RPM. */
    public void stop() {
        targetVelocityRPM = 0.0;
        leaderMotor.setControl(neutralRequest);
    }

    /**
     * Hot-reload Slot0 gains using refresh+apply to preserve current limits.
     */
    public void applyTuningConfig(double kS, double kV, double kA, double kP, double kI, double kD) {
        var configurator = leaderMotor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        configurator.refresh(config);

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;

        StatusCode status = configurator.apply(config);
        if (!status.isOK()) {
            System.err.println("Failed to apply flywheel tuning config: " + status);
        }
    }

    /**
     * Restore original Constants.java values by re-running configureLeaderMotor().
     */
    public void restoreDefaultConfig() {
        configureLeaderMotor();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Flywheel/Velocity RPM", getCurrentVelocityRPM());
        // SmartDashboard.putNumber("Flywheel/Target RPM", targetVelocityRPM);
        // SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}
