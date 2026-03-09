package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.generated.TurretLookupTables;

/**
 * Utility class that calculates the turret angle needed to point at a specific field target based on the robot's
 * current pose and alliance color.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;

    // Low-pass filter state for smoothing turret angle (0.0 = no change, 1.0 = no filtering)
    private static final double SMOOTHING_ALPHA = 0.15;
    private double filteredAngleDegrees = Double.NaN;

    /**
     * Result of an aiming calculation.
     *
     * @param turretAngleDegrees Robot-relative turret angle in degrees (-180 to +180)
     * @param distanceMeters Distance to target in meters
     * @param isReachable Whether the target is within valid shooting range
     */
    public record AimingResult(
            double turretAngleDegrees,
            double distanceMeters,
            boolean isReachable) {
    }

    /**
     * Creates a new TurretAimingCalculator.
     *
     * @param poseSupplier Supplier for the robot's current pose (e.g., drivetrain.getState()::Pose)
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
            // Re-normalize
            if (filteredAngleDegrees > 180.0)
                filteredAngleDegrees -= 360.0;
            else if (filteredAngleDegrees <= -180.0)
                filteredAngleDegrees += 360.0;
        }
        turretAngleDegrees = filteredAngleDegrees;

        // Check if target is within valid shooting range
        boolean isReachable = distanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        // Publish telemetry
        SmartDashboard.putNumber("TurretAim/TargetAngle", turretAngleDegrees);
        SmartDashboard.putNumber("TurretAim/Distance", distanceMeters);
        SmartDashboard.putBoolean("TurretAim/Reachable", isReachable);
        SmartDashboard.putString("TurretAim/Zone", getZoneName(robotPose));
        SmartDashboard.putString("TurretAim/Target",
                String.format("(%.1f, %.1f)", targetPosition.getX(), targetPosition.getY()));

        return new AimingResult(turretAngleDegrees, distanceMeters, isReachable);
    }

    /**
     * Gets the target position based on the current alliance and robot position on the field.
     *
     * <p>
     * When the robot is in its own scoring zone, aims at the alliance goal. When in the neutral or opponent zone, aims
     * at alliance-specific left/right targets based on robot Y position relative to the field centerline.
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
     * Gets the recommended hood angle based on distance to target. Uses interpolation from generated lookup table if
     * available, falls back to default constants if no calibration data exists.
     *
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        AimingResult result = calculate();
        double[][] table = TurretLookupTables.HOOD_LOOKUP_TABLE.length > 0
                ? TurretLookupTables.HOOD_LOOKUP_TABLE
                : TurretAimingConstants.HOOD_LOOKUP_TABLE;
        return interpolate(result.distanceMeters(), table);
    }

    /**
     * Gets the recommended flywheel RPM based on distance to target. Uses interpolation from generated lookup table if
     * available, falls back to default constants if no calibration data exists.
     *
     * @return Flywheel speed in RPM
     */
    public double getFlywheelRPM() {
        AimingResult result = calculate();
        double[][] table = TurretLookupTables.FLYWHEEL_LOOKUP_TABLE.length > 0
                ? TurretLookupTables.FLYWHEEL_LOOKUP_TABLE
                : TurretAimingConstants.FLYWHEEL_LOOKUP_TABLE;
        return interpolate(result.distanceMeters(), table);
    }

    /**
     * Linear interpolation from a lookup table.
     *
     * @param input The input value (e.g., distance)
     * @param lookupTable 2D array where [i][0] is input and [i][1] is output
     * @return Interpolated output value
     */
    private double interpolate(double input, double[][] lookupTable) {
        if (lookupTable.length == 0) {
            return 0.0;
        }

        // Clamp to table bounds
        if (input <= lookupTable[0][0]) {
            return lookupTable[0][1];
        }
        if (input >= lookupTable[lookupTable.length - 1][0]) {
            return lookupTable[lookupTable.length - 1][1];
        }

        // Find surrounding points and interpolate
        for (int i = 0; i < lookupTable.length - 1; i++) {
            if (input >= lookupTable[i][0] && input <= lookupTable[i + 1][0]) {
                double t = (input - lookupTable[i][0]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                return lookupTable[i][1] + t * (lookupTable[i + 1][1] - lookupTable[i][1]);
            }
        }

        return 0.0;
    }
}
