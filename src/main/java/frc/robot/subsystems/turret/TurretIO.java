package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean canCoderConnected = false;
    public double motorPositionDegrees = 0.0;
    public double canCoderPositionDegrees = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set turret position in encoder-space degrees (converted to rotations internally). */
  public default void setPosition(double encoderDegrees) {}

  /** Apply raw voltage output. */
  public default void setVoltage(double volts) {}

  /** Stop the turret motor. */
  public default void stop() {}

  /** Enable or disable soft limits. */
  public default void setSoftLimitsEnabled(boolean enabled) {}

  /** Update PID and Motion Magic gains. */
  public default void setPID(
      double kS,
      double kV,
      double kA,
      double kG,
      double kP,
      double kI,
      double kD,
      double cruiseVelocity,
      double acceleration,
      double jerk) {}
}
