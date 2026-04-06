package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean independentConnected = false;
    public boolean groupLeaderConnected = false;
    public boolean groupFollowerConnected = false;
    public double independentVelocityRPS = 0.0;
    public double independentOutputCurrent = 0.0;
    public double independentAppliedOutput = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Set only the independent motor duty cycle. */
  public default void setIndependentDutyCycle(double output) {}

  /** Set only the group motors duty cycle. */
  public default void setGroupDutyCycle(double output) {}

  /** Set all motors to the same duty cycle. */
  public default void setAllDutyCycle(double output) {}

  /** Stop all motors. */
  public default void stop() {}
}
