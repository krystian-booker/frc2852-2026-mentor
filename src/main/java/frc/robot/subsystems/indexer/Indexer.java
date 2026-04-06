package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  // Tunable jam detection parameters
  private final LoggedTunableNumber jamCurrentThreshold =
      new LoggedTunableNumber(
          "Indexer/JamCurrentThreshold", IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private final LoggedTunableNumber jamReverseDuration =
      new LoggedTunableNumber(
          "Indexer/JamReverseDuration", IndexerConstants.JAM_REVERSE_DURATION_SECONDS);
  private final LoggedTunableNumber jamCooldown =
      new LoggedTunableNumber("Indexer/JamCooldown", IndexerConstants.JAM_COOLDOWN_SECONDS);

  private final Alert independentDisconnected =
      new Alert("Indexer independent motor disconnected!", AlertType.kWarning);
  private final Alert groupLeaderDisconnected =
      new Alert("Indexer group leader motor disconnected!", AlertType.kWarning);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    independentDisconnected.set(!inputs.independentConnected);
    groupLeaderDisconnected.set(!inputs.groupLeaderConnected);

    Robot.batteryLogger.reportCurrentUsage(
        "Indexer/Independent", false, inputs.independentOutputCurrent);
  }

  /**
   * Command that feeds game pieces, automatically reversing only the independent motor on a current
   * spike to clear jams.
   */
  public Command feedCommand() {
    return run(() -> {
          io.setIndependentDutyCycle(IndexerConstants.FEED_SPEED);
          io.setGroupDutyCycle(IndexerConstants.FEED_SPEED);
        })
        .until(() -> inputs.independentOutputCurrent >= jamCurrentThreshold.get())
        .andThen(
            run(() -> io.setIndependentDutyCycle(IndexerConstants.REVERSE_SPEED))
                .withTimeout(jamReverseDuration.get()))
        .andThen(
            run(() -> {
                  io.setIndependentDutyCycle(IndexerConstants.FEED_SPEED);
                  io.setGroupDutyCycle(IndexerConstants.FEED_SPEED);
                })
                .withTimeout(jamCooldown.get()))
        .repeatedly()
        .finallyDo(this::stop);
  }

  public void runFeed() {
    io.setAllDutyCycle(IndexerConstants.FEED_SPEED);
  }

  public void runReverse() {
    io.setAllDutyCycle(IndexerConstants.REVERSE_SPEED);
  }

  public void stop() {
    io.stop();
  }

  public double getVelocityRPS() {
    return inputs.independentVelocityRPS;
  }

  public double getOutputCurrent() {
    return inputs.independentOutputCurrent;
  }
}
