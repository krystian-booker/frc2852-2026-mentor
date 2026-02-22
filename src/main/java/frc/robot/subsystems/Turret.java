package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.TurretAimingCalculator;
import frc.robot.util.TurretAimingCalculator.AimingResult;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {

    // Hardware
    private final TalonFX motor;
    private final CANcoder canCoder;

    // Control requests
    private final MotionMagicTorqueCurrentFOC positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Angle> canCoderPosition;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // State
    private double targetPositionDegrees = 0.0;

    public Turret() {
        // Initialize hardware
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        motor = new TalonFX(CANIds.TURRET_MOTOR, canBus);
        canCoder = new CANcoder(CANIds.TURRET_CANCODER, canBus);

        // Initialize control requests
        positionRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure hardware
        configureCANCoder();
        configureMotor();

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

        // Configure SysId routine for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Step voltage
                        null, // Use default timeout (10 s)
                        (state) -> SignalLogger.writeString("turret-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setControl(voltageRequest.withOutput(voltage.in(Volts))),
                        null,
                        this));
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Use FusedCANcoder for absolute position feedback
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
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
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_POSITION_DEGREES / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_POSITION_DEGREES / 360.0;

        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.SUPPLY_CURRENT_LOWER_TIME;

        config.TorqueCurrent.PeakForwardTorqueCurrent = TurretConstants.STATOR_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -TurretConstants.STATOR_CURRENT_LIMIT;

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

    public void setPosition(double degrees) {
        // Clamp to valid range
        targetPositionDegrees = Math.max(TurretConstants.MIN_POSITION_DEGREES,
                Math.min(TurretConstants.MAX_POSITION_DEGREES, degrees));

        // Convert degrees to rotations and command motion magic
        double rotations = targetPositionDegrees / 360.0;
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
        return motorPosition.getValue().in(Rotations) * 360.0;
    }

    public double getCANCoderPositionDegrees() {
        BaseStatusSignal.refreshAll(canCoderPosition);
        return canCoderPosition.getValue().in(Rotations) * 360.0;
    }

    public double getTargetPositionDegrees() {
        return targetPositionDegrees;
    }

    /**
     * Creates a command that continuously aims the turret at the target.
     * The turret will track the target position as the robot moves.
     *
     * @param calculator The TurretAimingCalculator to use for calculations
     * @return A command that continuously updates the turret position
     */
    public Command aimAtTargetCommand(TurretAimingCalculator calculator) {
        return run(() -> {
            AimingResult result = calculator.calculate();
            if (result.isReachable()) {
                setPosition(result.turretAngleDegrees());
            }
        }).withName("TurretAimAtTarget");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Turret/Position Degrees", getPositionDegrees());
        // SmartDashboard.putNumber("Turret/Target Degrees", targetPositionDegrees);
        // SmartDashboard.putNumber("Turret/CANCoder Degrees", getCANCoderPositionDegrees());
        // SmartDashboard.putBoolean("Turret/At Position", isAtPosition());
    }
}
