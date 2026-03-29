package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Diagnostic state — captures pre-filter angle and target info for logging
    private double lastRawAngleDegrees = Double.NaN;
    private Translation2d lastTargetPosition = new Translation2d();
    private double lastDistanceMeters = 0.0;

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

    public TurretAimingCalculator(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
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

        Translation2d targetPosition = getTargetPosition(robotPose);
        double turretAngleDegrees = calculateTurretAngleToTarget(robotPose, targetPosition);

        // Calculate distance for reachability check
        Translation2d turretPosition = getTurretFieldPosition(robotPose);
        double distanceMeters = turretPosition.getDistance(targetPosition);

        // Store for diagnostics
        lastTargetPosition = targetPosition;
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
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
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

    /** Returns the raw turret angle (before low-pass filter) from the last calculation. */
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
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
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
     * Uses bilinear interpolation from the 2D generated grid if available,
     * falls back to default constants if no calibration data exists.
     *
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        return gridLookup(getHoodGrid());
    }

    /**
     * Gets the recommended flywheel RPM based on robot field position.
     * Uses bilinear interpolation from the 2D generated grid if available,
     * falls back to default constants if no calibration data exists.
     *
     * @return Flywheel speed in RPM
     */
    public double getFlywheelRPM() {
        return gridLookup(getFlywheelGrid());
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

        double x = pose.getX();
        double y = pose.getY();

        // Calibration data is recorded for Blue alliance.
        // For Red alliance, mirror the position (field has 180° rotational symmetry).
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            x = CalibrationConstants.FIELD_LENGTH_METERS - x;
            y = CalibrationConstants.FIELD_WIDTH_METERS - y;
        }

        return bilinearInterpolate(x, y, grid);
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
}
