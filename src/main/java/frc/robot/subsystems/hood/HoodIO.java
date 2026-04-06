package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double positionDegrees = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Set hood position in mechanism degrees. */
  public default void setPosition(double degrees) {}

  /** Apply raw voltage output. */
  public default void setVoltage(double volts) {}

  /** Stop the hood motor. */
  public default void stop() {}

  /** Zero the motor encoder position. */
  public default void zeroEncoder() {}

  /** Enable or disable soft limits. */
  public default void setSoftLimitsEnabled(boolean forward, boolean reverse) {}

  /** Update PID and Motion Magic gains. */
  public default void setPID(
      double kS,
      double kV,
      double kA,
      double kG,
      double kP,
      double kI,
      double kD,
      double cruiseVelDegS,
      double accelDegS2,
      double jerkDegS3) {}
}
