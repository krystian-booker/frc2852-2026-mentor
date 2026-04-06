package frc.robot.subsystems.intakeactuator;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakeActuatorConstants;

public class IntakeActuatorIOSparkFlex implements IntakeActuatorIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  public IntakeActuatorIOSparkFlex() {
    motor = new SparkFlex(CANIds.INTAKE_ACTUATOR_MOTOR, MotorType.kBrushless);
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
    configureMotor();
    encoder.setPosition(0.0);
  }

  private void configureMotor() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.inverted(true);

    config.smartCurrentLimit(IntakeActuatorConstants.SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IntakeActuatorConstants.SECONDARY_CURRENT_LIMIT);

    config.encoder.positionConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / IntakeActuatorConstants.GEAR_RATIO / 60.0);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pid(
        IntakeActuatorConstants.KP, IntakeActuatorConstants.KI, IntakeActuatorConstants.KD);
    config.closedLoop.outputRange(
        IntakeActuatorConstants.MIN_OUTPUT, IntakeActuatorConstants.MAX_OUTPUT);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) {
        break;
      }
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure IntakeActuator motor: " + error);
    }
  }

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPS = encoder.getVelocity();
    inputs.appliedOutput = motor.getAppliedOutput();
    inputs.outputCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setPosition(double positionRotations) {
    closedLoopController.setSetpoint(positionRotations, SparkBase.ControlType.kPosition);
  }

  @Override
  public void setDutyCycle(double output) {
    motor.set(output);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public void zeroEncoder() {
    encoder.setPosition(0.0);
  }
}
