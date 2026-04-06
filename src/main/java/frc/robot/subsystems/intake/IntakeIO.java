package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double velocityRPS = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run both intake motors at the specified duty cycle (-1 to 1). */
  public default void setDutyCycle(double output) {}

  /** Stop both intake motors. */
  public default void stop() {}
}
