package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Logs mechanism component poses for AdvantageScope 3D visualization. Call {@link #update} each
 * loop with current mechanism positions, then call {@link #log} to publish.
 */
public class Mechanism3d {
  private static Mechanism3d instance;

  // Turret pivot position relative to robot center (from TurretAimingConstants)
  private static final Translation3d turretPivot =
      new Translation3d(
          Units.inchesToMeters(-4.719), // X offset
          Units.inchesToMeters(-5.250), // Y offset
          Units.inchesToMeters(10.0)); // Z height (estimated)

  // Hood pivot position relative to turret (estimated from CAD)
  private static final Translation3d hoodOffset =
      new Translation3d(0.0, 0.0, Units.inchesToMeters(4.0));

  private double turretAngleDegrees = 0.0;
  private double hoodAngleDegrees = 0.0;
  private double intakeActuatorRotations = 0.0;

  public static Mechanism3d getInstance() {
    if (instance == null) {
      instance = new Mechanism3d();
    }
    return instance;
  }

  public void setTurretAngle(double degrees) {
    this.turretAngleDegrees = degrees;
  }

  public void setHoodAngle(double degrees) {
    this.hoodAngleDegrees = degrees;
  }

  public void setIntakeActuatorPosition(double rotations) {
    this.intakeActuatorRotations = rotations;
  }

  /** Log all component poses to AdvantageScope. */
  public void log(String key) {
    // Turret rotates around Z axis
    Pose3d turretPose =
        new Pose3d(turretPivot, new Rotation3d(0.0, 0.0, Math.toRadians(turretAngleDegrees)));

    // Hood tilts relative to turret around Y axis
    Translation3d hoodPosition =
        turretPivot.plus(
            new Translation3d(
                hoodOffset.getX() * Math.cos(Math.toRadians(turretAngleDegrees)),
                hoodOffset.getX() * Math.sin(Math.toRadians(turretAngleDegrees)),
                hoodOffset.getZ()));
    Pose3d hoodPose =
        new Pose3d(
            hoodPosition,
            new Rotation3d(
                0.0,
                Math.toRadians(-hoodAngleDegrees), // Hood tilts forward (negative pitch)
                Math.toRadians(turretAngleDegrees)));

    Logger.recordOutput(key + "/Turret", turretPose);
    Logger.recordOutput(key + "/Hood", hoodPose);
    Logger.recordOutput(key + "/TurretAngleDeg", turretAngleDegrees);
    Logger.recordOutput(key + "/HoodAngleDeg", hoodAngleDegrees);
    Logger.recordOutput(key + "/IntakeActuatorRotations", intakeActuatorRotations);
  }
}
