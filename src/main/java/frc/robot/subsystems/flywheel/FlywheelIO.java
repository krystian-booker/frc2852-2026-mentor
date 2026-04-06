package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double velocityRPM = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the flywheel at the specified velocity in rotations per second. */
  public default void setVelocity(double rps) {}

  /** Apply raw voltage output. */
  public default void setVoltage(double volts) {}

  /** Apply raw torque current output. */
  public default void setCurrent(double amps) {}

  /** Stop the flywheel. */
  public default void stop() {}

  /** Update PID gains. */
  public default void setPID(double kS, double kV, double kA, double kP, double kI, double kD) {}
}
