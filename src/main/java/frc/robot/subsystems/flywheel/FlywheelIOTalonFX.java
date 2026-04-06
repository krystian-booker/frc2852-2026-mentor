package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.generated.TunerConstants;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Voltage> supplyVoltage;

  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FlywheelIOTalonFX() {
    leaderMotor = new TalonFX(CANIds.FLYWHEEL_LEADER_MOTOR, TunerConstants.kCANBus);
    followerMotor = new TalonFX(CANIds.FLYWHEEL_FOLLOWER_MOTOR, TunerConstants.kCANBus);

    configureLeaderMotor();
    configureFollowerMotor();

    leaderVelocity = leaderMotor.getVelocity();
    statorCurrent = leaderMotor.getStatorCurrent();
    supplyVoltage = leaderMotor.getSupplyVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.SIGNAL_UPDATE_FREQUENCY_HZ, leaderVelocity, statorCurrent, supplyVoltage);

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

    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
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

    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config, 0.25));

    followerMotor.setControl(
        new Follower(CANIds.FLYWHEEL_LEADER_MOTOR, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var leaderStatus = BaseStatusSignal.refreshAll(leaderVelocity, statorCurrent, supplyVoltage);
    inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerConnectedDebounce.calculate(true);
    inputs.velocityRPS = leaderVelocity.getValue().in(RotationsPerSecond);
    inputs.velocityRPM = inputs.velocityRPS * 60.0;
    inputs.statorCurrentAmps = statorCurrent.getValue().in(Amps);
    inputs.supplyVoltage = supplyVoltage.getValue().in(Volts);
    inputs.appliedVolts = leaderMotor.getMotorVoltage().refresh().getValue().in(Volts);
  }

  @Override
  public void setVelocity(double rps) {
    leaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setCurrent(double amps) {
    leaderMotor.setControl(torqueCurrentRequest.withOutput(amps));
  }

  @Override
  public void stop() {
    leaderMotor.setControl(neutralRequest);
  }

  @Override
  public void setPID(double kS, double kV, double kA, double kP, double kI, double kD) {
    var slot0 = new Slot0Configs();
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(slot0, 0.25));
  }
}
