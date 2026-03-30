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

import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ProjectileSimulator;
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

        this.shotCalculator = new ShotCalculator(config);

        // Build physics LUT based on 4" flywheel and estimated 2026 specs
        ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
                0.235, // ballMassKg
                0.1778, // ballDiameterM (7 inches)
                0.47, // dragCoeff
                0.2, // magnusCoeff
                1.225, // airDensityKgPerCubicM
                0.5, // exitHeightM
                0.1016, // flywheelDiameterM (4 inches)
                2.64, // targetHeightM (typical high goal)
                0.6, // slipFactor
                45.0, // baseLaunchAngleDeg
                0.001, // simDtSec
                1500, // minRPM
                6000, // maxRPM
                25, // maxIterations
                5.0 // maxTOFSamples
        );
        ProjectileSimulator sim = new ProjectileSimulator(params);

        // Sweep angles from 0 to 25 degrees
        ShotLUT lut = sim.generateVariableAngleShotLUT(0.0, 25.0, 1.0);
        shotCalculator.loadShotLUT(lut);
    }

    /**
     * Updates the SOTM solver. Because the Newton solver tracks velocity histories,
     * this MUST be called exactly once per robot cycle (20ms) from a periodic
     * block.
     */
    public void update() {
        Pose2d robotPose = poseSupplier.get();
        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            return;
        }

        ChassisSpeeds robotSpeeds = speedsSupplier.get();
        if (robotSpeeds == null)
            return;

        // Convert robot-relative speeds to field-relative
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        Translation2d targetPosition = getTargetPosition(robotPose);
        lastTargetPosition = targetPosition;

        // Unit vector pointing from target towards robot (hub forward)
        // This is used by the solver to prevent shooting through the hub.
        Translation2d displacement = robotPose.getTranslation().minus(targetPosition);
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

        if (shot.isValid() && shot.confidence() > 0) {
            lastTargetRPM = shot.rpm();
            lastTargetHoodAngle = shotCalculator.getHoodAngle(lastDistanceMeters);

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
