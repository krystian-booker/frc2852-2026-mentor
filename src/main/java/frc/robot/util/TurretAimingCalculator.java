package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TurretLookupTables;

/**
 * Utility class that calculates the turret angle needed to point at a specific
 * field target based on the robot's
 * current pose and alliance color.
 *
 * <p>
 * Uses grid lookup tables for hood angle and flywheel RPM based on position.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    // Alliance cache to prevent expensive DriverStation lookups
    private Alliance cachedAlliance = Alliance.Blue;
    private double lastAllianceCheckTime = -10.0;

    // Diagnostic state — captures pre-filter angle and target info for logging
    private double lastRawAngleDegrees = Double.NaN;
    private Translation2d lastTargetPosition = new Translation2d();
    private double lastDistanceMeters = 0.0;

    // SOTM cached state — shared between calculate() and
    // getHoodAngle()/getFlywheelRPM()
    private double lastTimeOfFlight = 0.0;
    private ChassisSpeeds lastFieldRelativeSpeeds = new ChassisSpeeds();

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

    public TurretAimingCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }

    /** Returns the current robot pose from the pose supplier. */
    public Pose2d getRobotPose() {
        return poseSupplier.get();
    }

    /**
     * Calculates the turret angle needed to aim at the alliance-specific target.
     *
     * @return AimingResult containing the turret angle, distance, and reachability
     */
    public AimingResult calculate() {
        Pose2d robotPose = poseSupplier.get();

        // Handle invalid pose
        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            return new AimingResult(0.0, 0.0, false);
        }

        // Convert robot-relative speeds to field-relative for SOTM
        ChassisSpeeds robotSpeeds = speedsSupplier.get();
        lastFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        Translation2d realTargetPosition = getTargetPosition(robotPose);
        Translation2d turretPosition = getTurretFieldPosition(robotPose);

        // Raw distance for time-of-flight estimation
        double rawDistance = turretPosition.getDistance(realTargetPosition);
        lastTimeOfFlight = estimateTimeOfFlight(rawDistance);

        // Determine the aiming target (virtual or real)
        Translation2d aimTarget = realTargetPosition;

        if (TurretAimingConstants.SOTM_ENABLED) {
            // Calculate how far the robot moves during the shot
            Translation2d robotMovement = new Translation2d(
                    lastFieldRelativeSpeeds.vxMetersPerSecond * lastTimeOfFlight,
                    lastFieldRelativeSpeeds.vyMetersPerSecond * lastTimeOfFlight);

            // Clamp lead distance for safety
            double leadDistance = robotMovement.getNorm();
            if (leadDistance > TurretAimingConstants.SOTM_MAX_LEAD_METERS) {
                double scale = TurretAimingConstants.SOTM_MAX_LEAD_METERS / leadDistance;
                robotMovement = new Translation2d(
                        robotMovement.getX() * scale,
                        robotMovement.getY() * scale);
            }

            // Shift target opposite to robot movement so the ball hits the real target
            aimTarget = realTargetPosition.minus(robotMovement);
        }

        double turretAngleDegrees = calculateTurretAngleToTarget(robotPose, aimTarget);

        // Distance to the aim target for reachability
        double distanceMeters = turretPosition.getDistance(aimTarget);

        // Store for diagnostics
        lastTargetPosition = aimTarget;
        lastDistanceMeters = distanceMeters;

        boolean isReachable = distanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        return new AimingResult(turretAngleDegrees, distanceMeters, isReachable);
    }

    /**
     * Gets the target position based on the current alliance and robot position on
     * the field.
     *
     * <p>
     * When the robot is in its own scoring zone, aims at the alliance goal. When in
     * the neutral or opponent zone, aims
     * at alliance-specific left/right targets based on robot Y position relative to
     * the field centerline.
     *
     * @param robotPose The robot's current pose
     * @return Target position in field coordinates (meters)
     */
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

    /**
     * Returns the raw turret angle (before low-pass filter) from the last
     * calculation.
     */
    public double getLastRawAngleDegrees() {
        return lastRawAngleDegrees;
    }

    /** Returns the target position used in the last calculation. */
    public Translation2d getLastTargetPosition() {
        return lastTargetPosition;
    }

    /** Returns the distance to target from the last calculation. */
    public double getLastDistanceMeters() {
        return lastDistanceMeters;
    }

    /**
     * Returns a human-readable zone name for dashboard telemetry.
     */
    private String getZoneName(Pose2d robotPose) {
        Alliance alliance = getAlliance();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        String side = robotY < TurretAimingConstants.FIELD_CENTERLINE_Y ? " Left" : " Right";

        if (alliance == Alliance.Blue) {
            if (robotX < TurretAimingConstants.BLUE_ZONE_MAX_X) {
                return "Blue Zone (Goal)";
            } else if (robotX > TurretAimingConstants.RED_ZONE_MIN_X) {
                return "Red Zone (Opponent)" + side;
            } else {
                return "Neutral Zone" + side;
            }
        } else {
            if (robotX > TurretAimingConstants.RED_ZONE_MIN_X) {
                return "Red Zone (Goal)";
            } else if (robotX < TurretAimingConstants.BLUE_ZONE_MAX_X) {
                return "Blue Zone (Opponent)" + side;
            } else {
                return "Neutral Zone" + side;
            }
        }
    }

    /**
     * Gets the recommended hood angle based on robot field position.
     * When SOTM is enabled, uses the effective (future) position where the robot
     * will be when the ball exits the barrel.
     *
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        if (TurretAimingConstants.SOTM_ENABLED) {
            Translation2d effectivePos = getEffectiveRobotPosition();
            return gridLookup(getHoodGrid(), effectivePos.getX(), effectivePos.getY());
        }
        return gridLookup(getHoodGrid());
    }

    /**
     * Gets the recommended flywheel RPM based on robot field position.
     * When SOTM is enabled, uses the effective (future) position where the robot
     * will be when the ball exits the barrel.
     *
     * @return Flywheel speed in RPM
     */
    public double getFlywheelRPM() {
        if (TurretAimingConstants.SOTM_ENABLED) {
            Translation2d effectivePos = getEffectiveRobotPosition();
            return gridLookup(getFlywheelGrid(), effectivePos.getX(), effectivePos.getY());
        }
        return gridLookup(getFlywheelGrid());
    }

    /**
     * Computes the effective robot position — where the robot will be when the ball
     * reaches the target, based on current velocity and estimated time of flight.
     */
    private Translation2d getEffectiveRobotPosition() {
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            return new Translation2d();
        }
        return pose.getTranslation().plus(new Translation2d(
                lastFieldRelativeSpeeds.vxMetersPerSecond * lastTimeOfFlight,
                lastFieldRelativeSpeeds.vyMetersPerSecond * lastTimeOfFlight));
    }

    /**
     * Returns the chassis angular velocity in degrees per second.
     * Used for turret rotation feedforward compensation.
     */
    public double getChassisOmegaDegreesPerSecond() {
        return Math.toDegrees(lastFieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Returns the active hood grid (generated or fallback).
     */
    private double[][] getHoodGrid() {
        return TurretLookupTables.HOOD_GRID.length > 0
                ? TurretLookupTables.HOOD_GRID
                : TurretAimingConstants.HOOD_GRID_FALLBACK;
    }

    /**
     * Returns the active flywheel grid (generated or fallback).
     */
    private double[][] getFlywheelGrid() {
        return TurretLookupTables.FLYWHEEL_GRID.length > 0
                ? TurretLookupTables.FLYWHEEL_GRID
                : TurretAimingConstants.FLYWHEEL_GRID_FALLBACK;
    }

    /**
     * Calculates the turret field position accounting for the mechanical offset
     * from robot center.
     */
    private Translation2d getTurretFieldPosition(Pose2d robotPose) {
        return robotPose.getTranslation()
                .plus(new Translation2d(
                        TurretAimingConstants.TURRET_OFFSET_X_METERS,
                        TurretAimingConstants.TURRET_OFFSET_Y_METERS)
                        .rotateBy(robotPose.getRotation()));
    }

    /**
     * Calculates the robot-relative turret angle to aim at a given target.
     * Includes turret offset, angle normalization, and range wrapping.
     *
     * @param robotPose      The robot's current pose
     * @param targetPosition The target to aim at (field coordinates)
     * @return Turret angle in degrees, within turret range [-225, +135]
     */
    private double calculateTurretAngleToTarget(Pose2d robotPose, Translation2d targetPosition) {
        Translation2d turretPosition = getTurretFieldPosition(robotPose);

        // Calculate field-relative angle to target
        double fieldAngleRadians = Math.atan2(
                targetPosition.getY() - turretPosition.getY(),
                targetPosition.getX() - turretPosition.getX());

        // Convert to robot-relative angle (subtract robot heading)
        double robotRelativeRadians = fieldAngleRadians - robotPose.getRotation().getRadians();

        // Normalize to (-180, +180) degrees
        double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;
        if (turretAngleDegrees > 180.0) {
            turretAngleDegrees -= 360.0;
        } else if (turretAngleDegrees <= -180.0) {
            turretAngleDegrees += 360.0;
        }

        // Wrap into turret's physical range [-225, +135]
        if (turretAngleDegrees > TurretConstants.MAX_POSITION_DEGREES) {
            turretAngleDegrees -= 360.0;
        }

        // Capture angle for diagnostics
        lastRawAngleDegrees = turretAngleDegrees;

        return turretAngleDegrees;
    }

    /**
     * Looks up a value from a 2D grid based on the robot's current field position.
     * Converts the robot pose to grid coordinates, mirrors for Red alliance,
     * and applies bilinear interpolation.
     *
     * @param grid 2D array indexed by [row][col]
     * @return Interpolated value at the robot's position
     */
    private double gridLookup(double[][] grid) {
        Pose2d pose = poseSupplier.get();
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())) {
            return grid[0][0];
        }
        return gridLookup(grid, pose.getX(), pose.getY());
    }

    /**
     * Looks up a value from a 2D grid at explicit field coordinates.
     * Mirrors for Red alliance and applies bilinear interpolation.
     *
     * @param grid   2D array indexed by [row][col]
     * @param fieldX Field X position in meters
     * @param fieldY Field Y position in meters
     * @return Interpolated value at the given position
     */
    private double gridLookup(double[][] grid, double fieldX, double fieldY) {
        double x = fieldX;
        double y = fieldY;

        // Calibration data is recorded for Blue alliance.
        // For Red alliance, mirror the position (field has 180° rotational symmetry).
        if (getAlliance() == Alliance.Red) {
            x = CalibrationConstants.FIELD_LENGTH_METERS - x;
            y = CalibrationConstants.FIELD_WIDTH_METERS - y;
        }

        return bilinearInterpolate(x, y, grid);
    }

    /**
     * Estimates the time of flight for a ball to reach the target.
     *
     * @param distanceMeters Distance to target in meters
     * @return Estimated time of flight in seconds
     */
    private double estimateTimeOfFlight(double distanceMeters) {
        return distanceMeters / TurretAimingConstants.AVERAGE_BALL_SPEED_MPS;
    }

    /**
     * Bilinear interpolation on a 2D grid.
     * Converts field (x, y) meters to continuous grid coordinates and interpolates
     * between the four surrounding cells.
     *
     * @param x    Field X position in meters (maps to grid column)
     * @param y    Field Y position in meters (maps to grid row)
     * @param grid 2D array indexed by [row][col]
     * @return Interpolated value
     */
    private double bilinearInterpolate(double x, double y, double[][] grid) {
        int rows = grid.length;
        int cols = grid[0].length;

        // Convert to continuous grid coordinates
        double gCol = x / CalibrationConstants.GRID_CELL_SIZE_METERS;
        double gRow = y / CalibrationConstants.GRID_CELL_SIZE_METERS;

        // Find surrounding cells and clamp to bounds
        int col0 = Math.max(0, Math.min((int) Math.floor(gCol), cols - 2));
        int row0 = Math.max(0, Math.min((int) Math.floor(gRow), rows - 2));
        int col1 = col0 + 1;
        int row1 = row0 + 1;

        // Fractional position within cell, clamped to [0, 1]
        double fx = Math.max(0.0, Math.min(1.0, gCol - col0));
        double fy = Math.max(0.0, Math.min(1.0, gRow - row0));

        // Bilinear interpolation
        double v00 = grid[row0][col0];
        double v01 = grid[row0][col1];
        double v10 = grid[row1][col0];
        double v11 = grid[row1][col1];

        return v00 * (1 - fx) * (1 - fy)
                + v01 * fx * (1 - fy)
                + v10 * (1 - fx) * fy
                + v11 * fx * fy;
    }

    /**
     * Retrieves the current alliance from the DriverStation, but throttles
     * queries to only once per second. Calling DriverStation.getAlliance()
     * frequently is a leading cause of 20ms loop overruns.
     */
    private Alliance getAlliance() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastAllianceCheckTime > 1.0) {
            cachedAlliance = DriverStation.getAlliance().orElse(cachedAlliance);
            lastAllianceCheckTime = currentTime;
        }
        return cachedAlliance;
    }
}
