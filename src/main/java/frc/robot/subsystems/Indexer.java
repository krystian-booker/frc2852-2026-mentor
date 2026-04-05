package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  // Independent motor (CAN 17) - reverses on jam to clear it
  private final SparkFlex independentMotor;
  // Group motors (CAN 18 leader, CAN 19 follower) - keep feeding during jam
  private final SparkFlex groupLeaderMotor;
  private final SparkFlex groupFollowerMotor;

  private final RelativeEncoder independentEncoder;

  public Indexer() {
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
      if (error == REVLibError.kOk) {
        break;
      }
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Independent motor: " + error);
    }
  }

  private void configureGroupLeaderMotor() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.inverted(true); // Inverted to match previous follower direction

    config.smartCurrentLimit(IndexerConstants.GROUP_SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IndexerConstants.GROUP_SECONDARY_CURRENT_LIMIT);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          groupLeaderMotor.configure(
              config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) {
        break;
      }
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Group Leader motor: " + error);
    }
  }

  private void configureGroupFollowerMotor() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.follow(CANIds.INDEXER_FOLLOWER_ONE_MOTOR, false); // Same direction as group leader

    config.smartCurrentLimit(IndexerConstants.GROUP_SMART_CURRENT_LIMIT);
    config.secondaryCurrentLimit(IndexerConstants.GROUP_SECONDARY_CURRENT_LIMIT);

    REVLibError error = REVLibError.kError;
    for (int i = 0; i < 5; i++) {
      error =
          groupFollowerMotor.configure(
              config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (error == REVLibError.kOk) {
        break;
      }
    }
    if (error != REVLibError.kOk) {
      System.err.println("Failed to configure Indexer Group Follower motor: " + error);
    }
  }

  /**
   * Command that feeds game pieces, automatically reversing only the independent motor on a current
   * spike to clear jams.
   */
  public Command feedCommand() {
    return run(() -> {
          independentMotor.set(IndexerConstants.FEED_SPEED);
          groupLeaderMotor.set(IndexerConstants.FEED_SPEED);
        })
        .until(
            () ->
                independentMotor.getOutputCurrent() >= IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS)
        .andThen(
            run(() -> independentMotor.set(IndexerConstants.REVERSE_SPEED))
                .withTimeout(IndexerConstants.JAM_REVERSE_DURATION_SECONDS))
        .andThen(
            run(() -> {
                  independentMotor.set(IndexerConstants.FEED_SPEED);
                  groupLeaderMotor.set(IndexerConstants.FEED_SPEED);
                })
                .withTimeout(IndexerConstants.JAM_COOLDOWN_SECONDS))
        .repeatedly()
        .finallyDo(this::stop);
  }

  /** Run all indexer motors at feed speed (no jam detection). */
  public void runFeed() {
    independentMotor.set(IndexerConstants.FEED_SPEED);
    groupLeaderMotor.set(IndexerConstants.FEED_SPEED);
  }

  /** Reverse all indexer motors. */
  public void runReverse() {
    independentMotor.set(IndexerConstants.REVERSE_SPEED);
    groupLeaderMotor.set(IndexerConstants.REVERSE_SPEED);
  }

  /** Stop all indexer motors. */
  public void stop() {
    independentMotor.setVoltage(0);
    groupLeaderMotor.setVoltage(0);
  }

  public double getVelocityRPS() {
    return independentEncoder.getVelocity();
  }

  public double getOutputCurrent() {
    return independentMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer/OutputCurrent", independentMotor.getOutputCurrent());
    SmartDashboard.putNumber("Indexer/AppliedOutput", independentMotor.getAppliedOutput());
    SmartDashboard.putNumber("Indexer/Velocity RPS", getVelocityRPS());
  }
}
