package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.SOTMConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TurretLookupTables;

/**
 * Utility class that calculates the turret angle needed to point at a specific
 * field target based on the robot's
 * current pose and alliance color.
 *
 * <p>
 * Supports shooting-on-the-move (SOTM) via the virtual target method:
 * compensates for robot velocity by aiming at a shifted target so the ball's
 * field-relative trajectory hits the real goal.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    // Low-pass filter state for velocity smoothing (SOTM)
    private double filteredVx = 0.0;
    private double filteredVy = 0.0;

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

    /**
     * Complete aiming solution including SOTM compensation.
     *
     * @param turretAngleDegrees Robot-relative turret angle (aims at virtual
     *                           target)
     * @param distanceMeters     Distance to virtual target in meters
     * @param isReachable        Whether the virtual target is within valid range
     * @param hoodAngleDegrees   SOTM-compensated hood angle
     * @param flywheelRPM        SOTM-compensated flywheel RPM
     * @param tofSeconds         Estimated time of flight
     * @param sotmActive         Whether SOTM correction was applied
     */
    public record SOTMAimingResult(
            double turretAngleDegrees,
            double distanceMeters,
            boolean isReachable,
            double hoodAngleDegrees,
            double flywheelRPM,
            double tofSeconds,
            boolean sotmActive) {
    }

    /**
     * Creates a new TurretAimingCalculator with SOTM support.
     *
     * @param poseSupplier   Supplier for the robot's current pose
     * @param speedsSupplier Supplier for the robot's current chassis speeds
     *                       (robot-relative)
     */
    public TurretAimingCalculator(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
    }

    /**
     * Calculates the turret angle needed to aim at the alliance-specific target.
     * Does NOT apply SOTM compensation — use {@link #calculateSOTM()} for that.
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
     * Calculates SOTM-compensated aiming solution using the virtual target method.
     *
     * <p>
     * When SOTM is disabled or robot speed is below threshold, falls back to
     * standard stationary aiming. Otherwise, iteratively computes a virtual target
     * that compensates for robot motion during ball flight.
     *
     * @return SOTMAimingResult with compensated turret angle, hood, flywheel, and
     *         TOF
     */
    public SOTMAimingResult calculateSOTM() {
        Pose2d robotPose = poseSupplier.get();

        // Handle invalid pose
        if (robotPose == null || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY())) {
            return new SOTMAimingResult(0.0, 0.0, false, getHoodAngle(), getFlywheelRPM(), 0.0, false);
        }

        // Convert robot-relative speeds to field-relative
        ChassisSpeeds robotSpeeds = speedsSupplier.get();
        double heading = robotPose.getRotation().getRadians();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double vxField = robotSpeeds.vxMetersPerSecond * cosH - robotSpeeds.vyMetersPerSecond * sinH;
        double vyField = robotSpeeds.vxMetersPerSecond * sinH + robotSpeeds.vyMetersPerSecond * cosH;

        // Smooth velocity
        filteredVx += SOTMConstants.VELOCITY_SMOOTHING_ALPHA * (vxField - filteredVx);
        filteredVy += SOTMConstants.VELOCITY_SMOOTHING_ALPHA * (vyField - filteredVy);

        double speed = Math.hypot(filteredVx, filteredVy);

        // Fall back to stationary aiming if SOTM disabled or below speed threshold
        if (!SOTMConstants.ENABLED || speed < SOTMConstants.MIN_SPEED_THRESHOLD) {
            AimingResult basic = calculate();
            publishSOTMTelemetry(false, 0.0, speed, 0.0, 0.0);
            return new SOTMAimingResult(
                    basic.turretAngleDegrees(), basic.distanceMeters(), basic.isReachable(),
                    getHoodAngle(), getFlywheelRPM(), 0.0, false);
        }

        Translation2d targetPosition = getTargetPosition(robotPose);
        Translation2d robotPosition = robotPose.getTranslation();

        // Initial TOF estimate using stationary flywheel RPM
        double flywheelRPM = gridLookup(getFlywheelGrid());
        double tof = estimateTOF(robotPosition, targetPosition, flywheelRPM);

        // Iterative TOF convergence
        double virtualPoseX = robotPose.getX();
        double virtualPoseY = robotPose.getY();

        for (int i = 0; i < SOTMConstants.TOF_ITERATIONS; i++) {
            // Virtual position: shift robot position in direction of travel
            virtualPoseX = robotPose.getX() + filteredVx * tof;
            virtualPoseY = robotPose.getY() + filteredVy * tof;

            // Virtual target: shift real target opposite to robot travel
            Translation2d virtualTarget = new Translation2d(
                    targetPosition.getX() - filteredVx * tof,
                    targetPosition.getY() - filteredVy * tof);

            // Refine TOF using flywheel RPM at virtual position
            flywheelRPM = gridLookupAt(virtualPoseX, virtualPoseY, getFlywheelGrid());
            tof = estimateTOF(robotPosition, virtualTarget, flywheelRPM);
        }

        // Final virtual target
        Translation2d virtualTarget = new Translation2d(
                targetPosition.getX() - filteredVx * tof,
                targetPosition.getY() - filteredVy * tof);

        // Look up hood and flywheel at virtual position
        double hoodAngle = gridLookupAt(virtualPoseX, virtualPoseY, getHoodGrid());
        flywheelRPM = gridLookupAt(virtualPoseX, virtualPoseY, getFlywheelGrid());

        // Aim turret at virtual target from actual position
        double turretAngleDegrees = calculateTurretAngleToTarget(robotPose, virtualTarget);

        // Distance for reachability
        Translation2d turretPosition = getTurretFieldPosition(robotPose);
        double distanceMeters = turretPosition.getDistance(virtualTarget);

        // Store for diagnostics
        lastTargetPosition = targetPosition;
        lastDistanceMeters = distanceMeters;

        boolean isReachable = distanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        publishSOTMTelemetry(true, tof, speed, virtualTarget.getX(), virtualTarget.getY());

        return new SOTMAimingResult(turretAngleDegrees, distanceMeters, isReachable,
                hoodAngle, flywheelRPM, tof, true);
    }

    /**
     * Publishes SOTM telemetry to SmartDashboard for tuning and debugging.
     */
    private void publishSOTMTelemetry(boolean active, double tof, double robotSpeed,
            double virtualTargetX, double virtualTargetY) {
        SmartDashboard.putBoolean("SOTM/Active", active);
        SmartDashboard.putNumber("SOTM/TOF", tof);
        SmartDashboard.putNumber("SOTM/RobotSpeed", robotSpeed);
        SmartDashboard.putNumber("SOTM/VirtualTargetX", virtualTargetX);
        SmartDashboard.putNumber("SOTM/VirtualTargetY", virtualTargetY);
        SmartDashboard.putNumber("SOTM/FilteredVx", filteredVx);
        SmartDashboard.putNumber("SOTM/FilteredVy", filteredVy);
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
     * Estimates time-of-flight from robot to target based on distance and flywheel
     * RPM.
     *
     * <p>
     * Uses: TOF = distance / (BALL_VELOCITY_FACTOR * flywheel_surface_speed)
     * where flywheel_surface_speed = PI * diameter * RPM / 60
     *
     * @param robotPosition Robot position on field
     * @param target        Target position on field
     * @param flywheelRPM   Current flywheel RPM
     * @return Estimated time of flight in seconds, clamped to MAX_TOF_SECONDS
     */
    private double estimateTOF(Translation2d robotPosition, Translation2d target, double flywheelRPM) {
        double distance = robotPosition.getDistance(target);
        double surfaceSpeed = Math.PI * SOTMConstants.FLYWHEEL_DIAMETER_METERS * flywheelRPM / 60.0;
        double ballSpeed = SOTMConstants.BALL_VELOCITY_FACTOR * surfaceSpeed;
        if (ballSpeed < 1.0) {
            ballSpeed = 1.0; // Prevent division by near-zero
        }
        double tof = distance / ballSpeed;
        return Math.min(tof, SOTMConstants.MAX_TOF_SECONDS);
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
     * Looks up a grid value at an explicit field position.
     * Used by SOTM to look up at the virtual position.
     * Handles Red alliance mirroring and field bounds clamping.
     *
     * @param x    Field X position in meters
     * @param y    Field Y position in meters
     * @param grid 2D array indexed by [row][col]
     * @return Interpolated value at the given position
     */
    private double gridLookupAt(double x, double y, double[][] grid) {
        // Mirror for Red alliance (same logic as gridLookup)
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            x = CalibrationConstants.FIELD_LENGTH_METERS - x;
            y = CalibrationConstants.FIELD_WIDTH_METERS - y;
        }

        // Clamp to field bounds
        x = Math.max(0, Math.min(CalibrationConstants.FIELD_LENGTH_METERS, x));
        y = Math.max(0, Math.min(CalibrationConstants.FIELD_WIDTH_METERS, y));

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
