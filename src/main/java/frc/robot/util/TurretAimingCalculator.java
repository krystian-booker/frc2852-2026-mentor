package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TurretLookupTables;

/**
 * Utility class that calculates the turret angle needed to point at a specific
 * field target based on the robot's
 * current pose and alliance color.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;

    // Low-pass filter state for smoothing turret angle (0.0 = no change, 1.0 = no
    // filtering)
    private static final double SMOOTHING_ALPHA = 0.15;
    private double filteredAngleDegrees = Double.NaN;

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

    /**
     * Creates a new TurretAimingCalculator.
     *
     * @param poseSupplier Supplier for the robot's current pose (e.g.,
     *                     drivetrain.getState()::Pose)
     */
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

        // Account for turret offset from robot center
        Translation2d turretPosition = robotPose.getTranslation()
                .plus(new Translation2d(
                        TurretAimingConstants.TURRET_OFFSET_X_METERS,
                        TurretAimingConstants.TURRET_OFFSET_Y_METERS)
                        .rotateBy(robotPose.getRotation()));

        // Calculate distance to target
        double distanceMeters = turretPosition.getDistance(targetPosition);

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

        // Smooth the angle with a low-pass filter to reduce jitter from pose noise
        if (Double.isNaN(filteredAngleDegrees)) {
            filteredAngleDegrees = turretAngleDegrees;
        } else {
            // Handle wrap-around: if the raw vs filtered differ by more than 180°, adjust
            double delta = turretAngleDegrees - filteredAngleDegrees;
            if (delta > 180.0)
                delta -= 360.0;
            else if (delta < -180.0)
                delta += 360.0;
            filteredAngleDegrees += SMOOTHING_ALPHA * delta;
            // Re-normalize to turret range
            if (filteredAngleDegrees > TurretConstants.MAX_POSITION_DEGREES)
                filteredAngleDegrees -= 360.0;
            else if (filteredAngleDegrees < TurretConstants.MIN_POSITION_DEGREES)
                filteredAngleDegrees += 360.0;
        }
        turretAngleDegrees = filteredAngleDegrees;

        // Check if target is within valid shooting range
        boolean isReachable = distanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        // Publish telemetry
        // SmartDashboard.putNumber("TurretAim/TargetAngle", turretAngleDegrees);
        // SmartDashboard.putNumber("TurretAim/Distance", distanceMeters);
        // SmartDashboard.putBoolean("TurretAim/Reachable", isReachable);
        // SmartDashboard.putString("TurretAim/Zone", getZoneName(robotPose));
        // SmartDashboard.putString("TurretAim/Target",
        // String.format("(%.1f, %.1f)", targetPosition.getX(), targetPosition.getY()));

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
        double[][] grid = TurretLookupTables.HOOD_GRID.length > 0
                ? TurretLookupTables.HOOD_GRID
                : TurretAimingConstants.HOOD_GRID_FALLBACK;
        return gridLookup(grid);
    }

    /**
     * Gets the recommended flywheel RPM based on robot field position.
     * Uses bilinear interpolation from the 2D generated grid if available,
     * falls back to default constants if no calibration data exists.
     *
     * @return Flywheel speed in RPM
     */
    public double getFlywheelRPM() {
        double[][] grid = TurretLookupTables.FLYWHEEL_GRID.length > 0
                ? TurretLookupTables.FLYWHEEL_GRID
                : TurretAimingConstants.FLYWHEEL_GRID_FALLBACK;
        return gridLookup(grid);
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
