package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSparkFlex implements IndexerIO {
  private final SparkFlex independentMotor;
  private final SparkFlex groupLeaderMotor;
  private final SparkFlex groupFollowerMotor;

  private final RelativeEncoder independentEncoder;

  public IndexerIOSparkFlex() {
    independentMotor = new SparkFlex(CANIds.INDEXER_LEADER_MOTOR, MotorType.kBrushless);
    groupLeaderMotor = new SparkFlex(CANIds.INDEXER_FOLLOWER_ONE_MOTOR, MotorType.kBrushless);
    groupFollowerMotor = new SparkFlex(CANIds.INDEXER_FOLLOWER_TWO_MOTOR, MotorType.kBrushless);

    independentEncoder = independentMotor.getEncoder();

    configureIndependentMotor();
    configureGroupLeaderMotor();
    configureGroupFollowerMotor();
  }

  private void configureIndependentMotor() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(IndexerConstants.SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IndexerConstants.SECONDARY_CURRENT_LIMIT);
    config.encoder.positionConversionFactor(1.0 / IndexerConstants.GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / IndexerConstants.GEAR_RATIO / 60.0);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          independentMotor.configure(
              config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) break;
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Independent motor: " + error);
    }
  }

  private void configureGroupLeaderMotor() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    config.smartCurrentLimit(IndexerConstants.GROUP_SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IndexerConstants.GROUP_SECONDARY_CURRENT_LIMIT);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          groupLeaderMotor.configure(
              config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) break;
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Group Leader motor: " + error);
    }
  }

  private void configureGroupFollowerMotor() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.follow(CANIds.INDEXER_FOLLOWER_ONE_MOTOR, false);
    config.smartCurrentLimit(IndexerConstants.GROUP_SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IndexerConstants.GROUP_SECONDARY_CURRENT_LIMIT);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          groupFollowerMotor.configure(
              config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) break;
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Group Follower motor: " + error);
    }
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.independentConnected = true;
    inputs.groupLeaderConnected = true;
    inputs.groupFollowerConnected = true;
    inputs.independentVelocityRPS = independentEncoder.getVelocity();
    inputs.independentOutputCurrent = independentMotor.getOutputCurrent();
    inputs.independentAppliedOutput = independentMotor.getAppliedOutput();
  }

  @Override
  public void setIndependentDutyCycle(double output) {
    independentMotor.set(output);
  }

  @Override
  public void setGroupDutyCycle(double output) {
    groupLeaderMotor.set(output);
  }

  @Override
  public void setAllDutyCycle(double output) {
    independentMotor.set(output);
    groupLeaderMotor.set(output);
  }

  @Override
  public void stop() {
    independentMotor.setVoltage(0);
    groupLeaderMotor.setVoltage(0);
  }
}
