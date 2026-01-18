package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretAimingConstants;

/**
 * Utility class that calculates the turret angle needed to point at a specific
 * field target based on the robot's current pose and alliance color.
 */
public class TurretAimingCalculator {
    private final Supplier<Pose2d> poseSupplier;

    /**
     * Result of an aiming calculation.
     *
     * @param turretAngleDegrees Robot-relative turret angle in degrees (0-360)
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

        Translation2d targetPosition = getTargetPosition();

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

        // Account for turret zero angle offset
        robotRelativeRadians -= Math.toRadians(TurretAimingConstants.TURRET_ZERO_ANGLE_DEGREES);

        // Normalize to [0, 360) degrees
        double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;
        if (turretAngleDegrees < 0) {
            turretAngleDegrees += 360.0;
        }

        // Check if target is within valid shooting range
        boolean isReachable = distanceMeters >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS;

        // Publish telemetry
        SmartDashboard.putNumber("TurretAim/TargetAngle", turretAngleDegrees);
        SmartDashboard.putNumber("TurretAim/Distance", distanceMeters);
        SmartDashboard.putBoolean("TurretAim/Reachable", isReachable);

        return new AimingResult(turretAngleDegrees, distanceMeters, isReachable);
    }

    /**
     * Gets the target position based on the current alliance.
     *
     * @return Target position in field coordinates (meters)
     */
    public Translation2d getTargetPosition() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
                ? TurretAimingConstants.RED_TARGET_POSITION
                : TurretAimingConstants.BLUE_TARGET_POSITION;
    }

    /**
     * Gets the recommended hood angle based on distance to target.
     * Uses interpolation from lookup table.
     *
     * @return Hood angle in degrees
     */
    public double getHoodAngle() {
        AimingResult result = calculate();
        return interpolate(result.distanceMeters(), TurretAimingConstants.HOOD_LOOKUP_TABLE);
    }

    /**
     * Gets the recommended flywheel RPM based on distance to target.
     * Uses interpolation from lookup table.
     *
     * @return Flywheel speed in RPM
     */
    public double getFlywheelRPM() {
        AimingResult result = calculate();
        return interpolate(result.distanceMeters(), TurretAimingConstants.FLYWHEEL_LOOKUP_TABLE);
    }

    /**
     * Linear interpolation from a lookup table.
     *
     * @param input      The input value (e.g., distance)
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
