package frc.robot.energy;

/** Centralized current limits for all subsystem motors. */
public class CurrentLimits {
  // Drive (per module)
  public static final double driveLimitAmps = 60.0;
  public static final double turnLimitAmps = 25.0;

  // Shooter
  public static final double flywheelLimitAmps = 60.0;
  public static final double hoodLimitAmps = 40.0;
  public static final double turretLimitAmps = 60.0;

  // Intake
  public static final double intakeLimitAmps = 40.0;
  public static final double intakeActuatorLimitAmps = 40.0;

  // Indexer
  public static final double indexerLimitAmps = 60.0;
  public static final double indexerGroupLimitAmps = 40.0;
}
