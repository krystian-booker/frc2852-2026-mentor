package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingCalculator.AimingResult;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {

    // Hardware
    private final TalonFX motor;
    private final CANcoder canCoder;

    // Control requests
    private final PositionVoltage positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Angle> canCoderPosition;

    // State
    private double targetPositionDegrees = 0.0;

    public Turret() {
        // Initialize hardware
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        motor = new TalonFX(CANIds.TURRET_MOTOR, canBus);
        canCoder = new CANcoder(CANIds.TURRET_CANCODER, canBus);

        // Initialize control requests
        positionRequest = new PositionVoltage(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure hardware
        configureCANCoder();
        configureMotor();

        // Seed motor position from CANcoder so they agree on startup
        // (avoids 360° wrapping mismatch between absolute position and fused position)
        motor.setPosition(canCoder.getAbsolutePosition().waitForUpdate(0.1).getValue().in(Rotations));

        // Cache status signals
        motorPosition = motor.getPosition();
        canCoderPosition = canCoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                motorPosition,
                canCoderPosition,
                motor.getMotorVoltage(),
                motor.getVelocity());

        // Optimize CAN bus utilization
        motor.optimizeBusUtilization();
        canCoder.optimizeBusUtilization();
    }

    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // CANcoder is 1:1 with turret rotation
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = canCoder.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure turret CANcoder: " + status);
        }
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Use FusedCANcoder for absolute position feedback
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        config.Feedback.FeedbackRemoteSensorID = CANIds.TURRET_CANCODER;
        config.Feedback.RotorToSensorRatio = TurretConstants.GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0; // CANcoder is 1:1 with turret

        // PID Gains
        config.Slot0.kS = TurretConstants.S;
        config.Slot0.kV = TurretConstants.V;
        config.Slot0.kA = TurretConstants.A;
        config.Slot0.kG = TurretConstants.G;
        config.Slot0.kP = TurretConstants.P;
        config.Slot0.kI = TurretConstants.I;
        config.Slot0.kD = TurretConstants.D;

        // Motion Magic configuration
        config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = TurretConstants.MOTION_MAGIC_JERK;

        // Software limits to prevent over-rotation (wire wrap protection)
        // Use encoder-space limits so soft limits stay at the same physical positions
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (TurretConstants.ENCODER_MAX_DEGREES
                + TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (TurretConstants.ENCODER_MIN_DEGREES
                - TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.SUPPLY_CURRENT_LOWER_TIME;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure turret motor: " + status);
        }
    }

    public void setPosition(double aimDegrees) {
        // Wrap into turret range before clamping
        if (aimDegrees > TurretConstants.MAX_POSITION_DEGREES) {
            aimDegrees -= 360.0;
        } else if (aimDegrees < TurretConstants.MIN_POSITION_DEGREES) {
            aimDegrees += 360.0;
        }
        // Safety clamp (should rarely trigger after wrapping)
        targetPositionDegrees = Math.max(TurretConstants.MIN_POSITION_DEGREES,
                Math.min(TurretConstants.MAX_POSITION_DEGREES, aimDegrees));

        // Convert aiming degrees to encoder degrees, then to rotations
        double encoderDegrees = targetPositionDegrees + TurretConstants.FORWARD_ENCODER_POSITION_DEGREES;
        double rotations = encoderDegrees / 360.0;
        motor.setControl(positionRequest.withPosition(rotations));
    }

    public void stop() {
        motor.setControl(neutralRequest);
        targetPositionDegrees = getPositionDegrees();
    }

    public boolean isAtPosition() {
        double currentDegrees = getPositionDegrees();
        return Math.abs(currentDegrees - targetPositionDegrees) < TurretConstants.POSITION_TOLERANCE_DEGREES;
    }

    public boolean isAtPosition(double degrees) {
        double currentDegrees = getPositionDegrees();
        return Math.abs(currentDegrees - degrees) < TurretConstants.POSITION_TOLERANCE_DEGREES;
    }

    public double getPositionDegrees() {
        BaseStatusSignal.refreshAll(motorPosition);
        double encoderDegrees = motorPosition.getValue().in(Rotations) * 360.0;
        return encoderDegrees - TurretConstants.FORWARD_ENCODER_POSITION_DEGREES;
    }

    public double getCANCoderPositionDegrees() {
        BaseStatusSignal.refreshAll(canCoderPosition);
        return canCoderPosition.getValue().in(Rotations) * 360.0;
    }

    public double getTargetPositionDegrees() {
        return targetPositionDegrees;
    }

    /**
     * Creates a command that continuously aims the turret at the target. The turret
     * will track the target position as
     * the robot moves.
     *
     * @param calculator The TurretAimingCalculator to use for calculations
     * @return A command that continuously updates the turret position
     */
    public Command aimAtTargetCommand(AimingCalculator calculator) {
        return run(() -> {
            AimingResult result = calculator.calculate();
            if (result.isReachable()) {
                setPosition(result.turretAngleDegrees());
            }
        }).withName("TurretAimAtTarget");
    }

    /**
     * Creates a command that holds the turret pointing at a fixed field-relative
     * heading. As the robot rotates, the
     * turret counter-rotates to maintain the same field direction. Captures the
     * current field heading on
     * initialization.
     *
     * @param robotHeadingSupplier supplies the robot's field-relative heading in
     *                             degrees
     */
    public Command fieldHoldCommand(java.util.function.DoubleSupplier robotHeadingSupplier) {
        double[] fieldTarget = new double[1]; // captured on init
        return runOnce(() -> {
            // Capture the field-relative direction the turret is currently pointing
            fieldTarget[0] = getPositionDegrees() + robotHeadingSupplier.getAsDouble();
        }).andThen(run(() -> {
            double turretAngle = fieldTarget[0] - robotHeadingSupplier.getAsDouble();
            // Normalize to [-180, 180]
            turretAngle = ((turretAngle % 360.0) + 540.0) % 360.0 - 180.0;
            // Wrap into turret range
            if (turretAngle > TurretConstants.MAX_POSITION_DEGREES) {
                turretAngle -= 360.0;
            }
            setPosition(turretAngle);
        })).withName("TurretFieldHold");
    }

    /**
     * Applies a small positive voltage to test motor direction. Watch the CANcoder:
     * if position INCREASES, direction is
     * correct. If position DECREASES, flip motor inversion or CANcoder direction.
     */
    public void testDirectionPositive() {
        motor.setControl(voltageRequest.withOutput(1.0)); // Small positive voltage
    }

    /**
     * Applies a small negative voltage to test motor direction.
     */
    public void testDirectionNegative() {
        motor.setControl(voltageRequest.withOutput(-1.0)); // Small negative voltage
    }

    /** Apply raw voltage output for characterization. */
    public void applyVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    /** Disable CTRE soft limits for sysid (free rotation with wires removed). */
    public void disableSoftLimits() {
        var config = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = false;
        config.ReverseSoftLimitEnable = false;
        motor.getConfigurator().apply(config);
    }

    /** Re-enable CTRE soft limits after sysid. */
    public void enableSoftLimits() {
        var config = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = true;
        config.ForwardSoftLimitThreshold = (TurretConstants.ENCODER_MAX_DEGREES
                + TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
        config.ReverseSoftLimitEnable = true;
        config.ReverseSoftLimitThreshold = (TurretConstants.ENCODER_MIN_DEGREES
                - TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
        motor.getConfigurator().apply(config);
    }

    /** Get mechanism velocity in rotations per second (signal already at 250Hz). */
    public double getVelocityRPS() {
        return motor.getVelocity().refresh().getValue().in(RotationsPerSecond);
    }

    /** Read applied motor voltage for telemetry. */
    public double getMotorVoltage() {
        return motor.getMotorVoltage().refresh().getValue().in(Volts);
    }

    /** Read stator current for diagnostics. */
    public double getStatorCurrent() {
        return motor.getStatorCurrent().refresh().getValue().in(Amps);
    }

    /**
     * Hot-reload gains using refresh+apply to preserve soft limits and current
     * limits.
     */
    public void applyTuningConfig(double kS, double kV, double kA, double kG,
            double kP, double kI, double kD,
            double cruiseVelocity, double acceleration, double jerk) {
        var configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        configurator.refresh(config);

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        config.MotionMagic.MotionMagicJerk = jerk;

        StatusCode status = configurator.apply(config);
        if (!status.isOK()) {
            System.err.println("Failed to apply tuning config: " + status);
        }
    }

    /** Restore original Constants.java values by re-running configureMotor(). */
    public void restoreDefaultConfig() {
        configureMotor();
    }

    /**
     * Commands a small movement relative to current position for safe testing.
     */
    public void nudge(double deltaDegrees) {
        double current = getPositionDegrees();
        setPosition(current + deltaDegrees);
    }

    /** Get the current encoder-space position in degrees (for debugging). */
    public double getEncoderDegrees() {
        BaseStatusSignal.refreshAll(motorPosition);
        return motorPosition.getValue().in(Rotations) * 360.0;
    }

    @Override
    public void periodic() {
        // double position = getPositionDegrees();
        // SmartDashboard.putNumber("Turret/Position Degrees", position);
        // SmartDashboard.putNumber("Turret/Target Degrees", targetPositionDegrees);
        // SmartDashboard.putBoolean("Turret/At Position", isAtPosition());

        // double encoderDeg = getEncoderDegrees();
        // double canCoderDeg = getCANCoderPositionDegrees();
        // SmartDashboard.putNumber("Turret/Encoder Degrees", encoderDeg);
        // SmartDashboard.putNumber("Turret/CANCoder Degrees", canCoderDeg);
        // SmartDashboard.putNumber("Turret/CANCoder Raw Rotations",
        // canCoderPosition.refresh().getValue().in(Rotations));
        // SmartDashboard.putNumber("Turret/Motor Raw Rotations",
        // motorPosition.refresh().getValue().in(Rotations));
        // SmartDashboard.putNumber("Turret/Motor Stator Current",
        // motor.getStatorCurrent().refresh().getValue().in(Amps));
        // SmartDashboard.putNumber("Turret/Motor Voltage",
        // motor.getMotorVoltage().refresh().getValue().in(Volts));
        // SmartDashboard.putBoolean("Turret/In Overshoot Zone",
        // position < TurretConstants.MIN_POSITION_DEGREES || position >
        // TurretConstants.MAX_POSITION_DEGREES);
        // SmartDashboard.putNumber("Turret/Distance From Limit",
        // Math.min(position - TurretConstants.MIN_POSITION_DEGREES,
        // TurretConstants.MAX_POSITION_DEGREES - position));
    }
}
