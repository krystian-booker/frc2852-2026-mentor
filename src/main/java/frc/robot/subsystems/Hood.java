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
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import frc.robot.Constants.HoodConstants;

import static edu.wpi.first.units.Units.*;

public class Hood extends SubsystemBase {

    // Hardware
    private final TalonFX motor;
    private final CANcoder cancoder;

    // Control requests
    private final MotionMagicTorqueCurrentFOC positionRequest;
    private final NeutralOut neutralRequest;
    private final VoltageOut voltageRequest;

    // Status signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Angle> cancoderPosition;

    // SysId routine for characterization
    private final SysIdRoutine sysIdRoutine;

    // State
    private double targetPositionDegrees = 0.0;

    public Hood() {
        // Initialize hardware
        CANBus canBus = new CANBus(CANIds.CANIVORE);
        motor = new TalonFX(CANIds.HOOD_MOTOR, canBus);
        cancoder = new CANcoder(CANIds.HOOD_CANCODER, canBus);

        // Initialize control requests
        positionRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        neutralRequest = new NeutralOut();
        voltageRequest = new VoltageOut(0);

        // Configure hardware
        configureCANCoder();
        configureMotor();

        // Cache status signals
        motorPosition = motor.getPosition();
        cancoderPosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
                motorPosition,
                cancoderPosition,
                motor.getVelocity(),
                motor.getMotorVoltage());

        // Optimize CAN bus utilization
        motor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();

        // Configure SysId routine for characterization
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Use 4V for position mechanism
                        null, // Use default timeout (10 s)
                        (state) -> SignalLogger.writeString("hood-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setControl(voltageRequest.withOutput(voltage.in(Volts))),
                        null,
                        this));
    }

    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = HoodConstants.CANCODER_OFFSET;

        // Apply with retry
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = cancoder.getConfigurator().apply(config);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.err.println("Failed to configure hood CANCoder: " + status);
        }
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Feedback - use FusedCANcoder for absolute positioning
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = CANIds.HOOD_CANCODER;
        config.Feedback.RotorToSensorRatio = HoodConstants.GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0;

        // PID Gains (Slot 0)
        config.Slot0.kS = HoodConstants.S;
        config.Slot0.kV = HoodConstants.V;
        config.Slot0.kA = HoodConstants.A;
        config.Slot0.kP = HoodConstants.P;
        config.Slot0.kI = HoodConstants.I;
        config.Slot0.kD = HoodConstants.D;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = 0.0; // Tune based on hood weight

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

        // Torque current limits
        config.TorqueCurrent.PeakForwardTorqueCurrent = HoodConstants.STATOR_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -HoodConstants.STATOR_CURRENT_LIMIT;

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

    public double getCANCoderPositionDegrees() {
        BaseStatusSignal.refreshAll(cancoderPosition);
        return cancoderPosition.getValue().in(Rotations) * 360.0;
    }

    public void setNeutral() {
        motor.setControl(neutralRequest);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/Position Degrees", getCurrentPositionDegrees());
        SmartDashboard.putNumber("Hood/Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Hood/CANCoder Degrees", getCANCoderPositionDegrees());
        SmartDashboard.putBoolean("Hood/At Position", atPosition());
    }
}
