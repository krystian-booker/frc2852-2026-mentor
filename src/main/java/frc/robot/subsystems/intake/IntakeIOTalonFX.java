package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Current> leftStatorCurrent;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    leftMotor = new TalonFX(CANIds.INTAKE_LEFT_MOTOR);
    rightMotor = new TalonFX(CANIds.INTAKE_RIGHT_MOTOR);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

    leftVelocity = leftMotor.getVelocity();
    leftStatorCurrent = leftMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.SIGNAL_UPDATE_FREQUENCY_HZ, leftVelocity, leftStatorCurrent);

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();
  }

  private void configureMotor(TalonFX motor, InvertedValue inversion) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inversion;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SUPPLY_CURRENT_LOWER_TIME;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var leftStatus = BaseStatusSignal.refreshAll(leftVelocity, leftStatorCurrent);
    inputs.leftConnected = leftConnectedDebounce.calculate(leftStatus.isOK());
    inputs.rightConnected = rightConnectedDebounce.calculate(true);
    inputs.velocityRPS = leftVelocity.getValue().in(RotationsPerSecond);
    inputs.statorCurrentAmps = leftStatorCurrent.getValue().in(Amps);
  }

  @Override
  public void setDutyCycle(double output) {
    leftMotor.setControl(dutyRequest.withOutput(output));
    rightMotor.setControl(dutyRequest.withOutput(output));
  }

  @Override
  public void stop() {
    leftMotor.setControl(neutralRequest);
    rightMotor.setControl(neutralRequest);
  }
}
