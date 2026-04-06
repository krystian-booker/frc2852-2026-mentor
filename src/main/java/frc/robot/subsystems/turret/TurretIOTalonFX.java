package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final CANcoder canCoder;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<Angle> canCoderPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer canCoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TurretIOTalonFX() {
    motor = new TalonFX(CANIds.TURRET_MOTOR, TunerConstants.kCANBus);
    canCoder = new CANcoder(CANIds.TURRET_CANCODER, TunerConstants.kCANBus);

    configureCANCoder();
    configureMotor();

    // Seed motor position from CANcoder
    motor.setPosition(canCoder.getAbsolutePosition().getValue().in(Rotations));

    motorPosition = motor.getPosition();
    canCoderPosition = canCoder.getAbsolutePosition();
    motorVelocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
        motorPosition,
        canCoderPosition,
        motorVelocity,
        motorVoltage,
        statorCurrent);

    motor.optimizeBusUtilization();
    canCoder.optimizeBusUtilization();
  }

  private void configureCANCoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET;

    tryUntilOk(5, () -> canCoder.getConfigurator().apply(config, 0.25));
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    config.Feedback.FeedbackRemoteSensorID = CANIds.TURRET_CANCODER;
    config.Feedback.RotorToSensorRatio = TurretConstants.GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0.kS = TurretConstants.S;
    config.Slot0.kV = TurretConstants.V;
    config.Slot0.kA = TurretConstants.A;
    config.Slot0.kG = TurretConstants.G;
    config.Slot0.kP = TurretConstants.P;
    config.Slot0.kI = TurretConstants.I;
    config.Slot0.kD = TurretConstants.D;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = TurretConstants.MOTION_MAGIC_JERK;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        (TurretConstants.ENCODER_MAX_DEGREES + TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        (TurretConstants.ENCODER_MIN_DEGREES - TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = TurretConstants.SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = TurretConstants.SUPPLY_CURRENT_LOWER_TIME;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, statorCurrent);
    var canCoderStatus = BaseStatusSignal.refreshAll(canCoderPosition);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.canCoderConnected = canCoderConnectedDebounce.calculate(canCoderStatus.isOK());
    inputs.motorPositionDegrees = motorPosition.getValue().in(Rotations) * 360.0;
    inputs.canCoderPositionDegrees = canCoderPosition.getValue().in(Rotations) * 360.0;
    inputs.velocityRPS = motorVelocity.getValue().in(RotationsPerSecond);
    inputs.appliedVolts = motorVoltage.getValue().in(Volts);
    inputs.statorCurrentAmps = statorCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double encoderDegrees) {
    motor.setControl(positionRequest.withPosition(encoderDegrees / 360.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
    softLimits.ForwardSoftLimitEnable = enabled;
    softLimits.ForwardSoftLimitThreshold =
        (TurretConstants.ENCODER_MAX_DEGREES + TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
    softLimits.ReverseSoftLimitEnable = enabled;
    softLimits.ReverseSoftLimitThreshold =
        (TurretConstants.ENCODER_MIN_DEGREES - TurretConstants.SOFT_LIMIT_BUFFER_DEGREES) / 360.0;
    motor.getConfigurator().apply(softLimits);
  }

  @Override
  public void setPID(
      double kS,
      double kV,
      double kA,
      double kG,
      double kP,
      double kI,
      double kD,
      double cruiseVelocity,
      double acceleration,
      double jerk) {
    var slot0 = new Slot0Configs();
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    var mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseVelocity;
    mm.MotionMagicAcceleration = acceleration;
    mm.MotionMagicJerk = jerk;

    tryUntilOk(5, () -> motor.getConfigurator().apply(slot0, 0.25));
    tryUntilOk(5, () -> motor.getConfigurator().apply(mm, 0.25));
  }
}
