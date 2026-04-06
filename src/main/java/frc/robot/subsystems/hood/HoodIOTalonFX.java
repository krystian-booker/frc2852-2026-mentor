package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.HoodConstants;
import frc.robot.generated.TunerConstants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOTalonFX() {
    motor = new TalonFX(CANIds.HOOD_MOTOR, TunerConstants.kCANBus);

    configureMotor();
    motor.setPosition(0.0);

    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.SIGNAL_UPDATE_FREQUENCY_HZ,
        motorPosition,
        motorVelocity,
        motorVoltage,
        statorCurrent);

    motor.optimizeBusUtilization();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;

    config.Slot0.kS = HoodConstants.S;
    config.Slot0.kV = HoodConstants.V;
    config.Slot0.kA = HoodConstants.A;
    config.Slot0.kP = HoodConstants.P;
    config.Slot0.kI = HoodConstants.I;
    config.Slot0.kD = HoodConstants.D;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = HoodConstants.G;

    config.MotionMagic.MotionMagicCruiseVelocity =
        HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY / 360.0;
    config.MotionMagic.MotionMagicAcceleration = HoodConstants.MOTION_MAGIC_ACCELERATION / 360.0;
    config.MotionMagic.MotionMagicJerk = HoodConstants.MOTION_MAGIC_JERK / 360.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.MAX_POSITION_DEGREES / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.MIN_POSITION_DEGREES / 360.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.SUPPLY_CURRENT_LOWER_TIME;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, statorCurrent);
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionDegrees = motorPosition.getValue().in(Rotations) * 360.0;
    inputs.velocityRPS = motorVelocity.getValue().in(RotationsPerSecond);
    inputs.appliedVolts = motorVoltage.getValue().in(Volts);
    inputs.statorCurrentAmps = statorCurrent.getValue().in(Amps);
  }

  @Override
  public void setPosition(double degrees) {
    motor.setControl(positionRequest.withPosition(degrees / 360.0));
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
  public void zeroEncoder() {
    motor.setPosition(0.0);
  }

  @Override
  public void setSoftLimitsEnabled(boolean forward, boolean reverse) {
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
    softLimits.ForwardSoftLimitEnable = forward;
    softLimits.ForwardSoftLimitThreshold = HoodConstants.MAX_POSITION_DEGREES / 360.0;
    softLimits.ReverseSoftLimitEnable = reverse;
    softLimits.ReverseSoftLimitThreshold = HoodConstants.MIN_POSITION_DEGREES / 360.0;
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
      double cruiseVelDegS,
      double accelDegS2,
      double jerkDegS3) {
    var slot0 = new Slot0Configs();
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    var mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseVelDegS / 360.0;
    mm.MotionMagicAcceleration = accelDegS2 / 360.0;
    mm.MotionMagicJerk = jerkDegS3 / 360.0;

    tryUntilOk(5, () -> motor.getConfigurator().apply(slot0, 0.25));
    tryUntilOk(5, () -> motor.getConfigurator().apply(mm, 0.25));
  }
}
