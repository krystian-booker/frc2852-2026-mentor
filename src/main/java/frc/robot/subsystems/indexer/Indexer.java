package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

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
        .until(() -> inputs.independentOutputCurrent >= IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS)
        .andThen(
            run(() -> io.setIndependentDutyCycle(IndexerConstants.REVERSE_SPEED))
                .withTimeout(IndexerConstants.JAM_REVERSE_DURATION_SECONDS))
        .andThen(
            run(() -> {
                  io.setIndependentDutyCycle(IndexerConstants.FEED_SPEED);
                  io.setGroupDutyCycle(IndexerConstants.FEED_SPEED);
                })
                .withTimeout(IndexerConstants.JAM_COOLDOWN_SECONDS))
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
