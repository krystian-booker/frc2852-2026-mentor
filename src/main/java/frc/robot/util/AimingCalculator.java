package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.GeneratedShotLUT;
import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ShotLUT;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class that calculates the turret angle needed to point at a specific field target based
 * on the robot's current pose and alliance color.
 *
 * <p>This implementation uses the frc-fire-control physics-based SOTM solver.
 */
public class AimingCalculator {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final ShotCalculator shotCalculator;
  private final ShotCalculator.Config config;

  // Live-tunable SOTM parameters (show up under /Tuning/SOTM/ in NetworkTables)
  private final LoggedTunableNumber sotmDragCoeff = new LoggedTunableNumber("SOTM/DragCoeff", 0.24);
  private final LoggedTunableNumber phaseDelayMs =
      new LoggedTunableNumber("SOTM/PhaseDelayMs", 30.0);
  private final LoggedTunableNumber mechLatencyMs =
      new LoggedTunableNumber("SOTM/MechLatencyMs", 20.0);
  private final LoggedTunableNumber maxSOTMSpeed =
      new LoggedTunableNumber("SOTM/MaxSOTMSpeed", 3.0);
  private final LoggedTunableNumber rpmOffset = new LoggedTunableNumber("SOTM/RPMOffset", 0.0);

  // Alliance cache to prevent expensive DriverStation lookups
  private Alliance cachedAlliance = Alliance.Blue;
  private double lastAllianceCheckTime = -10.0;

  // Debug logging throttle (print every N cycles = every N*20ms)
  private int updateCycleCount = 0;
  private static final int LOG_EVERY_N_CYCLES = 50; // every 1 second

  // Diagnostic/Cached state
  private double lastRawAngleDegrees = 0.0;
  private Translation2d lastTargetPosition = new Translation2d();
  private double lastDistanceMeters = 0.0;
  private double lastTargetRPM = 0.0;
  private double lastTargetHoodAngle = 0.0;
  private double lastSotmConfidence = 0.0;
  private boolean isReachable = false;

  /**
   * Result of an aiming calculation.
   *
   * @param turretAngleDegrees Robot-relative turret angle in degrees (-180 to +180)
   * @param distanceMeters Distance to target in meters
   * @param isReachable Whether the target is within valid shooting range
   */
  public record AimingResult(
      double turretAngleDegrees, double distanceMeters, boolean isReachable) {}

  public AimingCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;

    this.config = new ShotCalculator.Config();
    config.launcherOffsetX = TurretAimingConstants.TURRET_OFFSET_X_METERS;
    config.launcherOffsetY = TurretAimingConstants.TURRET_OFFSET_Y_METERS;
    config.maxScoringDistance = 17.0;
    config.headingMaxErrorRad = Math.PI; // Turret aims independently; disable body-heading penalty
    config.headingSpeedScalar = 0.0; // Turret: no speed-based tightening of heading tolerance
    config.headingReferenceDistance =
        1000.0; // Turret: force distanceScale to max (no distance tightening)

    this.shotCalculator = new ShotCalculator(config);

    // Load pre-generated LUT (built at compile time by tools/GenerateShotLUT.java)
    ShotLUT lut = GeneratedShotLUT.create();
    shotCalculator.loadShotLUT(lut);

    // --- DEBUG: LUT summary ---
    System.out.println("[AimingCalc] LUT loaded with " + lut.size() + " pre-generated entries");
    double[] sampleDists = {1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0, 15.0, 17.0};
    for (double d : sampleDists) {
      var sp = lut.get(d);
      System.out.printf(
          "[AimingCalc] LUT @ %.1fm: RPM=%.0f, mechAngle=%.1f, TOF=%.3fs%n",
          d, sp.rpm(), sp.angleDeg(), sp.tofSec());
    }
  }

  /**
   * Updates the SOTM solver. Because the Newton solver tracks velocity histories, this MUST be
   * called exactly once per robot cycle (20ms) from a periodic block.
   */
  public void update() {
    updateCycleCount++;
    boolean shouldLog = (updateCycleCount % LOG_EVERY_N_CYCLES) == 0;

    // Apply live-tunable SOTM parameters
    config.sotmDragCoeff = sotmDragCoeff.get();
    config.phaseDelayMs = phaseDelayMs.get();
    config.mechLatencyMs = mechLatencyMs.get();
    config.maxSOTMSpeed = maxSOTMSpeed.get();

    // RPM offset: reset then set to desired value (adjustOffset accumulates)
    double desiredOffset = rpmOffset.get();
    double currentOffset = shotCalculator.getOffset();
    if (desiredOffset != currentOffset) {
      shotCalculator.resetOffset();
      shotCalculator.adjustOffset(desiredOffset);
    }

    Pose2d robotPose = poseSupplier.get();
    if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
      if (shouldLog) System.out.println("[AimingCalc] SKIP: pose null or NaN");
      return;
    }

    ChassisSpeeds robotSpeeds = speedsSupplier.get();
    if (robotSpeeds == null) {
      if (shouldLog) System.out.println("[AimingCalc] SKIP: speeds null");
      return;
    }

    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

    Translation2d targetPosition = getTargetPosition(robotPose);
    lastTargetPosition = targetPosition;

    // Unit vector pointing from robot towards target (hub forward)
    // This is used by the solver to prevent shooting through the hub.
    Translation2d displacement = targetPosition.minus(robotPose.getTranslation());
    double dist = Math.max(0.1, displacement.getNorm());
    Translation2d hubForward = displacement.div(dist);

    ShotCalculator.ShotInputs inputs =
        new ShotCalculator.ShotInputs(
            robotPose,
            fieldSpeeds,
            robotSpeeds,
            targetPosition,
            hubForward,
            1.0 // Vision confidence placeholder
            );

    ShotCalculator.LaunchParameters shot = shotCalculator.calculate(inputs);

    lastDistanceMeters = shot.solvedDistanceM();
    lastSotmConfidence = shot.confidence();

    isReachable =
        lastDistanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
            && lastDistanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

    if (shouldLog) {
      System.out.printf(
          "[AimingCalc] pose=(%.2f,%.2f) heading=%.1f target=(%.2f,%.2f) dist=%.2fm%n",
          robotPose.getX(),
          robotPose.getY(),
          robotPose.getRotation().getDegrees(),
          targetPosition.getX(),
          targetPosition.getY(),
          dist);
      System.out.printf(
          "[AimingCalc] shot: valid=%b conf=%.1f rpm=%.0f projDist=%.2f solvedDist=%.2f iters=%d%n",
          shot.isValid(),
          shot.confidence(),
          shot.rpm(),
          shot.projectedDistanceM(),
          shot.solvedDistanceM(),
          shot.iterationsUsed());
    }

    // Always update turret tracking angle when the solver found a valid geometric solution.
    // Confidence gates SHOOTING (RPM/hood), not TRACKING (turret angle).
    if (shot.isValid()) {
      // Convert absolute field drive angle to a robot-relative turret angle
      double robotRelativeRadians =
          shot.driveAngle().getRadians() - robotPose.getRotation().getRadians();
      double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;

      if (turretAngleDegrees > 180.0) turretAngleDegrees -= 360.0;
      else if (turretAngleDegrees <= -180.0) turretAngleDegrees += 360.0;

      if (turretAngleDegrees > TurretConstants.MAX_POSITION_DEGREES) {
        turretAngleDegrees -= 360.0;
      }

      lastRawAngleDegrees = turretAngleDegrees;

      // Only update shooting parameters when confidence is sufficient
      if (shot.confidence() > 0) {
        lastTargetRPM = shot.rpm();
        lastTargetHoodAngle = shotCalculator.getHoodAngle(shot.projectedDistanceM());
      }

      if (shouldLog) {
        System.out.printf(
            "[AimingCalc] OUTPUT: RPM=%.0f hoodAngle=%.1f turretAngle=%.1f conf=%.1f%n",
            lastTargetRPM, lastTargetHoodAngle, turretAngleDegrees, shot.confidence());
      }
    } else {
      // Solver returned INVALID (e.g. speed cap exceeded, behind hub).
      // Fall back to pure geometric aim from turret position (not robot center).
      double heading = robotPose.getRotation().getRadians();
      double cosH = Math.cos(heading);
      double sinH = Math.sin(heading);
      double turretFieldX =
          robotPose.getX()
              + TurretAimingConstants.TURRET_OFFSET_X_METERS * cosH
              - TurretAimingConstants.TURRET_OFFSET_Y_METERS * sinH;
      double turretFieldY =
          robotPose.getY()
              + TurretAimingConstants.TURRET_OFFSET_X_METERS * sinH
              + TurretAimingConstants.TURRET_OFFSET_Y_METERS * cosH;
      double geometricAngleRad =
          Math.atan2(targetPosition.getY() - turretFieldY, targetPosition.getX() - turretFieldX);
      double robotRelativeRadians = geometricAngleRad - robotPose.getRotation().getRadians();
      double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;

      if (turretAngleDegrees > 180.0) turretAngleDegrees -= 360.0;
      else if (turretAngleDegrees <= -180.0) turretAngleDegrees += 360.0;

      if (turretAngleDegrees > TurretConstants.MAX_POSITION_DEGREES) {
        turretAngleDegrees -= 360.0;
      }

      lastRawAngleDegrees = turretAngleDegrees;
      lastDistanceMeters = dist;
      isReachable =
          dist >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
              && dist <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

      if (shouldLog) {
        System.out.printf(
            "[AimingCalc] FALLBACK AIM: turretAngle=%.1f dist=%.2f (shot invalid: valid=%b conf=%.1f)%n",
            turretAngleDegrees, dist, shot.isValid(), shot.confidence());
      }
    }

    // ── AdvantageKit telemetry ──────────────────────────────────────
    // Selected target on 2D field view
    Logger.recordOutput("Aiming/TargetPosition", new Pose2d(lastTargetPosition, new Rotation2d()));

    // Aim line: robot to target (renders as connected path on 2D field)
    Logger.recordOutput(
        "Aiming/AimLine",
        new Pose2d[] {robotPose, new Pose2d(lastTargetPosition, new Rotation2d())});

    // All target positions (static reference markers on field)
    Logger.recordOutput(
        "Aiming/BlueTargets",
        new Pose2d[] {
          new Pose2d(TurretAimingConstants.BLUE_TARGET_POSITION, new Rotation2d()),
          new Pose2d(TurretAimingConstants.BLUE_LEFT_TARGET_POSITION, new Rotation2d()),
          new Pose2d(TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION, new Rotation2d())
        });

    // Turret aim direction as a pose (robot position + field-relative turret heading)
    double aimFieldAngle =
        robotPose.getRotation().getRadians() + Math.toRadians(lastRawAngleDegrees);
    Logger.recordOutput(
        "Aiming/TurretAimPose",
        new Pose3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), 1.0),
            new Rotation3d(0, 0, aimFieldAngle)));

    // Numeric diagnostics
    Logger.recordOutput("Aiming/TurretAngleDeg", lastRawAngleDegrees);
    Logger.recordOutput("Aiming/DistanceMeters", lastDistanceMeters);
    Logger.recordOutput("Aiming/FlywheelRPM", lastTargetRPM);
    Logger.recordOutput("Aiming/HoodAngleDeg", lastTargetHoodAngle);
    Logger.recordOutput("Aiming/Confidence", lastSotmConfidence);
    Logger.recordOutput("Aiming/IsReachable", isReachable);

    // Target name for quick identification
    String targetName = identifyTargetName(lastTargetPosition);
    Logger.recordOutput("Aiming/TargetName", targetName);
  }

  /**
   * Returns the cached results of the last update() calculation. Use this in the Turret command
   * instead of actively calculating.
   */
  public AimingResult calculate() {
    return new AimingResult(lastRawAngleDegrees, lastDistanceMeters, isReachable);
  }

  public Translation2d getTargetPosition(Pose2d robotPose) {
    // Work in blue-side coordinates, then flip result if red alliance
    double blueX =
        AllianceFlipUtil.shouldFlip()
            ? AllianceFlipUtil.applyX(robotPose.getX())
            : robotPose.getX();
    double blueY =
        AllianceFlipUtil.shouldFlip()
            ? AllianceFlipUtil.applyY(robotPose.getY())
            : robotPose.getY();

    Translation2d blueTarget;
    if (blueX < TurretAimingConstants.BLUE_ZONE_MAX_X) {
      blueTarget = TurretAimingConstants.BLUE_TARGET_POSITION;
    } else if (blueY < TurretAimingConstants.FIELD_CENTERLINE_Y) {
      blueTarget = TurretAimingConstants.BLUE_LEFT_TARGET_POSITION;
    } else {
      blueTarget = TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION;
    }

    return AllianceFlipUtil.apply(blueTarget);
  }

  public double getLastRawAngleDegrees() {
    return lastRawAngleDegrees;
  }

  public Translation2d getLastTargetPosition() {
    return lastTargetPosition;
  }

  public double getLastDistanceMeters() {
    return lastDistanceMeters;
  }

  public double getSotmConfidence() {
    return lastSotmConfidence;
  }

  public double getHoodAngle() {
    return lastTargetHoodAngle;
  }

  public double getFlywheelRPM() {
    return lastTargetRPM;
  }

  /** Identify which target is currently selected by comparing positions. */
  private String identifyTargetName(Translation2d target) {
    Translation2d bluePrimary = AllianceFlipUtil.apply(TurretAimingConstants.BLUE_TARGET_POSITION);
    Translation2d blueLeft =
        AllianceFlipUtil.apply(TurretAimingConstants.BLUE_LEFT_TARGET_POSITION);
    Translation2d blueRight =
        AllianceFlipUtil.apply(TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION);

    if (target.getDistance(bluePrimary) < 0.01) return "Primary";
    if (target.getDistance(blueLeft) < 0.01) return "Left";
    if (target.getDistance(blueRight) < 0.01) return "Right";
    return "Unknown";
  }
}
