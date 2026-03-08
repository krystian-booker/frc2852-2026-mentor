package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.HoodConstants;

import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase {

    // Hardware
    private final TalonFX motor;

    // Control requests
    private final MotionMagicVoltage positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<Angle> motorPosition;

    // State
    private double targetPositionDegrees = 0.0;

    public Hood() {
        // Initialize hardware
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        motor = new TalonFX(CANIds.HOOD_MOTOR, canBus);

        // Initialize control requests
        positionRequest = new MotionMagicVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure hardware
        configureMotor();
        motor.setPosition(0.0);

        // Cache status signals
        motorPosition = motor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                motorPosition,
                motor.getVelocity(),
                motor.getMotorVoltage());

        // Optimize CAN bus utilization
        motor.optimizeBusUtilization();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Feedback - use rotor sensor, seeded to 0 on startup
        config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;

        // PID Gains (Slot 0)
        config.Slot0.kS = HoodConstants.S;
        config.Slot0.kV = HoodConstants.V;
        config.Slot0.kA = HoodConstants.A;
        config.Slot0.kP = HoodConstants.P;
        config.Slot0.kI = HoodConstants.I;
        config.Slot0.kD = HoodConstants.D;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = HoodConstants.G;

        // Motion Magic - convert deg/s to rot/s
        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY / 360.0;
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.MOTION_MAGIC_ACCELERATION / 360.0;
        config.MotionMagic.MotionMagicJerk = HoodConstants.MOTION_MAGIC_JERK / 360.0;

        // Soft limits - convert degrees to rotations
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_POSITION_DEGREES / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_POSITION_DEGREES / 360.0;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.SUPPLY_CURRENT_LOWER_TIME;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure hood motor: " + status);
        }
    }

    public void setPosition(double degrees) {
        targetPositionDegrees = clamp(degrees, HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES);
        double rotations = targetPositionDegrees / 360.0;
        motor.setControl(positionRequest.withPosition(rotations));
    }

    public boolean atPosition() {
        double currentDegrees = getCurrentPositionDegrees();
        return Math.abs(currentDegrees - targetPositionDegrees) < HoodConstants.POSITION_TOLERANCE_DEGREES;
    }

    public double getCurrentPositionDegrees() {
        BaseStatusSignal.refreshAll(motorPosition);
        return motorPosition.getValue().in(Rotations) * 360.0;
    }

    public void setNeutral() {
        motor.setControl(neutralRequest);
    }

    /**
     * Applies a small positive voltage to test motor direction. Watch the dashboard: if position INCREASES, direction
     * is correct. If position DECREASES, flip motor inversion.
     */
    public void testDirectionPositive() {
        motor.setControl(voltageRequest.withOutput(1.0));
    }

    /**
     * Applies a small negative voltage to test motor direction.
     */
    public void testDirectionNegative() {
        motor.setControl(voltageRequest.withOutput(-1.0));
    }

    /**
     * Commands a small movement relative to current position for safe testing.
     */
    public void nudge(double deltaDegrees) {
        double current = getCurrentPositionDegrees();
        setPosition(current + deltaDegrees);
    }

    /** Apply raw voltage output for characterization. */
    public void applyVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    /** Get mechanism velocity in rotations per second. */
    public double getVelocityRPS() {
        return motor.getVelocity().refresh().getValue().in(RotationsPerSecond);
    }

    /** Get motor stator current in Amps. */
    public double getStatorCurrent() {
        return motor.getStatorCurrent().refresh().getValue().in(Amps);
    }

    /**
     * Hot-reload only Slot0 and MotionMagic gains. Uses per-group config objects
     * so gear ratio, soft limits, and current limits are never touched.
     * Motion Magic params are in deg/s (matching HoodConstants convention), divided by 360 internally.
     */
    public void applyTuningConfig(double kS, double kV, double kA, double kG,
            double kP, double kI, double kD,
            double cruiseVelDegS, double accelDegS2, double jerkDegS3) {
        var configurator = motor.getConfigurator();

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kG = kG;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = cruiseVelDegS / 360.0;
        mm.MotionMagicAcceleration = accelDegS2 / 360.0;
        mm.MotionMagicJerk = jerkDegS3 / 360.0;

        StatusCode status = configurator.apply(slot0);
        if (!status.isOK()) {
            System.err.println("Failed to apply hood Slot0 config: " + status);
        }
        status = configurator.apply(mm);
        if (!status.isOK()) {
            System.err.println("Failed to apply hood MotionMagic config: " + status);
        }
    }

    /** Restore original Constants.java values by re-running configureMotor(). */
    public void restoreDefaultConfig() {
        configureMotor();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        double position = getCurrentPositionDegrees();
        SmartDashboard.putNumber("Hood/Position Degrees", position);
        SmartDashboard.putNumber("Hood/Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Hood/Motor Raw Rotations", motorPosition.refresh().getValue().in(Rotations));
        SmartDashboard.putNumber("Hood/Motor Stator Current", motor.getStatorCurrent().refresh().getValue().in(Amps));
        SmartDashboard.putNumber("Hood/Motor Voltage", motor.getMotorVoltage().refresh().getValue().in(Volts));
        SmartDashboard.putBoolean("Hood/At Position", atPosition());
    }
}
