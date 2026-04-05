package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.GeneratedShotLUT;

import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ShotLUT;

/**
 * Utility class that calculates the turret angle needed to point at a specific
 * field target based on the robot's current pose and alliance color.
 *
 * <p>
 * This implementation uses the frc-fire-control physics-based SOTM solver.
 */
public class AimingCalculator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final ShotCalculator shotCalculator;

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
     * @param turretAngleDegrees Robot-relative turret angle in degrees (-180 to
     *                           +180)
     * @param distanceMeters     Distance to target in meters
     * @param isReachable        Whether the target is within valid shooting range
     */
    public record AimingResult(
            double turretAngleDegrees,
            double distanceMeters,
            boolean isReachable) {
    }

    public AimingCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;

        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = TurretAimingConstants.TURRET_OFFSET_X_METERS;
        config.launcherOffsetY = TurretAimingConstants.TURRET_OFFSET_Y_METERS;
        config.maxScoringDistance = 17.0;
        config.headingMaxErrorRad = Math.PI; // Turret aims independently; disable body-heading penalty

        this.shotCalculator = new ShotCalculator(config);

        // Load pre-generated LUT (built at compile time by tools/GenerateShotLUT.java)
        ShotLUT lut = GeneratedShotLUT.create();
        shotCalculator.loadShotLUT(lut);

        // --- DEBUG: LUT summary ---
        System.out.println("[AimingCalc] LUT loaded with " + lut.size() + " pre-generated entries");
        double[] sampleDists = {1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0, 15.0, 17.0};
        for (double d : sampleDists) {
            var sp = lut.get(d);
            System.out.printf("[AimingCalc] LUT @ %.1fm: RPM=%.0f, mechAngle=%.1f, TOF=%.3fs%n",
                    d, sp.rpm(), sp.angleDeg(), sp.tofSec());
        }
    }

    /**
     * Updates the SOTM solver. Because the Newton solver tracks velocity histories,
     * this MUST be called exactly once per robot cycle (20ms) from a periodic
     * block.
     */
    public void update() {
        updateCycleCount++;
        boolean shouldLog = (updateCycleCount % LOG_EVERY_N_CYCLES) == 0;

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
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        Translation2d targetPosition = getTargetPosition(robotPose);
        lastTargetPosition = targetPosition;

        // Unit vector pointing from robot towards target (hub forward)
        // This is used by the solver to prevent shooting through the hub.
        Translation2d displacement = targetPosition.minus(robotPose.getTranslation());
        double dist = Math.max(0.1, displacement.getNorm());
        Translation2d hubForward = displacement.div(dist);

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
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

        isReachable = lastDistanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && lastDistanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        if (shouldLog) {
            System.out.printf("[AimingCalc] pose=(%.2f,%.2f) heading=%.1f target=(%.2f,%.2f) dist=%.2fm%n",
                    robotPose.getX(), robotPose.getY(),
                    robotPose.getRotation().getDegrees(),
                    targetPosition.getX(), targetPosition.getY(), dist);
            System.out.printf("[AimingCalc] shot: valid=%b conf=%.1f rpm=%.0f projDist=%.2f solvedDist=%.2f iters=%d%n",
                    shot.isValid(), shot.confidence(), shot.rpm(),
                    shot.projectedDistanceM(), shot.solvedDistanceM(), shot.iterationsUsed());
        }

        if (shot.isValid() && shot.confidence() > 0) {
            lastTargetRPM = shot.rpm();
            lastTargetHoodAngle = shotCalculator.getHoodAngle(shot.projectedDistanceM());

            // Convert absolute field drive angle to a robot-relative turret angle
            double robotRelativeRadians = shot.driveAngle().getRadians() - robotPose.getRotation().getRadians();
            double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;

            if (turretAngleDegrees > 180.0)
                turretAngleDegrees -= 360.0;
            else if (turretAngleDegrees <= -180.0)
                turretAngleDegrees += 360.0;

            if (turretAngleDegrees > TurretConstants.MAX_POSITION_DEGREES) {
                turretAngleDegrees -= 360.0;
            }

            lastRawAngleDegrees = turretAngleDegrees;

            if (shouldLog) {
                System.out.printf("[AimingCalc] OUTPUT: RPM=%.0f hoodAngle=%.1f turretAngle=%.1f%n",
                        lastTargetRPM, lastTargetHoodAngle, turretAngleDegrees);
            }
        } else {
            if (shouldLog) {
                System.out.printf("[AimingCalc] SHOT REJECTED: valid=%b confidence=%.1f (RPM/hood unchanged at %.0f/%.1f)%n",
                        shot.isValid(), shot.confidence(), lastTargetRPM, lastTargetHoodAngle);
            }
        }
    }

    /**
     * Returns the cached results of the last update() calculation.
     * Use this in the Turret command instead of actively calculating.
     */
    public AimingResult calculate() {
        return new AimingResult(lastRawAngleDegrees, lastDistanceMeters, isReachable);
    }

    public Translation2d getTargetPosition(Pose2d robotPose) {
        Alliance alliance = getAlliance();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        if (alliance == Alliance.Blue) {
            if (robotX < TurretAimingConstants.BLUE_ZONE_MAX_X) {
                return TurretAimingConstants.BLUE_TARGET_POSITION;
            } else if (robotY < TurretAimingConstants.FIELD_CENTERLINE_Y) {
                return TurretAimingConstants.BLUE_LEFT_TARGET_POSITION;
            } else {
                return TurretAimingConstants.BLUE_RIGHT_TARGET_POSITION;
            }
        } else {
            if (robotX > TurretAimingConstants.RED_ZONE_MIN_X) {
                return TurretAimingConstants.RED_TARGET_POSITION;
            } else if (robotY < TurretAimingConstants.FIELD_CENTERLINE_Y) {
                return TurretAimingConstants.RED_LEFT_TARGET_POSITION;
            } else {
                return TurretAimingConstants.RED_RIGHT_TARGET_POSITION;
            }
        }
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

    private Alliance getAlliance() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastAllianceCheckTime > 1.0) {
            cachedAlliance = DriverStation.getAlliance().orElse(cachedAlliance);
            lastAllianceCheckTime = currentTime;
        }
        return cachedAlliance;
    }
}
