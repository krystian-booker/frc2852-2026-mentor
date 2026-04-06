package frc.robot.subsystems.intakeactuator;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeActuatorIO {
  @AutoLog
  public static class IntakeActuatorIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double appliedOutput = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeActuatorIOInputs inputs) {}

  /** Set position setpoint in mechanism rotations. */
  public default void setPosition(double positionRotations) {}

  /** Set duty cycle output (-1 to 1). */
  public default void setDutyCycle(double output) {}

  /** Stop the motor. */
  public default void stop() {}

  /** Zero the encoder position. */
  public default void zeroEncoder() {}
}
