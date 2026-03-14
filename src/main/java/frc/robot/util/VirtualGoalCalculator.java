package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.ShootOnTheMoveConstants;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.generated.TurretLookupTables;

/**
 * Calculates a "virtual goal" position that compensates for robot motion while shooting.
 *
 * <p>When the robot is moving, the ball inherits the robot's velocity vector, causing misses.
 * This calculator shifts the aiming target opposite to robot motion by an amount proportional
 * to the estimated time-of-flight, so the ball arrives at the real goal despite robot movement.
 *
 * <p>When the robot is stationary (below velocity threshold), the real goal is returned unchanged.
 */
public class VirtualGoalCalculator {

    /**
     * Result of a virtual goal calculation.
     *
     * @param virtualGoal         The compensated target position in field coordinates
     * @param virtualDistanceMeters Distance from turret to the virtual goal
     * @param virtualAngleRadians  Field-relative angle from turret to virtual goal
     * @param timeOfFlightSeconds  Estimated ball time-of-flight
     * @param sotmActive           Whether SOTM compensation was applied
     */
    public record VirtualGoalResult(
            Translation2d virtualGoal,
            double virtualDistanceMeters,
            double virtualAngleRadians,
            double timeOfFlightSeconds,
            boolean sotmActive) {
    }

    public VirtualGoalCalculator() {
        SmartDashboard.putBoolean("SOTM/Enabled", ShootOnTheMoveConstants.DEFAULT_ENABLED);
    }

    /**
     * Calculates the virtual goal position that compensates for robot motion.
     *
     * @param turretPosition Field-relative turret position (meters)
     * @param realGoal       Real target position in field coordinates (meters)
     * @param fieldSpeeds    Field-relative robot velocity
     * @param realDistance   Distance from turret to real goal (meters)
     * @return VirtualGoalResult with compensated target info
     */
    public VirtualGoalResult calculate(Translation2d turretPosition, Translation2d realGoal,
            ChassisSpeeds fieldSpeeds, double realDistance) {

        boolean enabled = SmartDashboard.getBoolean("SOTM/Enabled", ShootOnTheMoveConstants.DEFAULT_ENABLED);
        double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        // No compensation if disabled or robot is nearly stationary
        if (!enabled || robotSpeed < ShootOnTheMoveConstants.MIN_VELOCITY_THRESHOLD_MPS) {
            double angle = Math.atan2(
                    realGoal.getY() - turretPosition.getY(),
                    realGoal.getX() - turretPosition.getX());

            SmartDashboard.putBoolean("SOTM/Active", false);
            SmartDashboard.putNumber("SOTM/TimeOfFlight", 0.0);
            SmartDashboard.putNumber("SOTM/VirtualGoalX", realGoal.getX());
            SmartDashboard.putNumber("SOTM/VirtualGoalY", realGoal.getY());
            SmartDashboard.putNumber("SOTM/VirtualDistance", realDistance);

            return new VirtualGoalResult(realGoal, realDistance, angle, 0.0, false);
        }

        // Iterative convergence: refine virtual goal using updated LUT values
        double currentDistance = realDistance;
        Translation2d virtualGoal = realGoal;
        double tof = 0.0;

        for (int i = 0; i < ShootOnTheMoveConstants.CONVERGENCE_ITERATIONS; i++) {
            double rpm = interpolate(currentDistance, getFlywheelTable());
            double hoodAngleDeg = interpolate(currentDistance, getHoodTable());
            tof = estimateTimeOfFlight(rpm, hoodAngleDeg, currentDistance);

            // Shift target opposite to robot motion
            virtualGoal = new Translation2d(
                    realGoal.getX() - fieldSpeeds.vxMetersPerSecond * tof,
                    realGoal.getY() - fieldSpeeds.vyMetersPerSecond * tof);

            currentDistance = turretPosition.getDistance(virtualGoal);
        }

        double virtualAngle = Math.atan2(
                virtualGoal.getY() - turretPosition.getY(),
                virtualGoal.getX() - turretPosition.getX());

        // Telemetry
        SmartDashboard.putBoolean("SOTM/Active", true);
        SmartDashboard.putNumber("SOTM/TimeOfFlight", tof);
        SmartDashboard.putNumber("SOTM/VirtualGoalX", virtualGoal.getX());
        SmartDashboard.putNumber("SOTM/VirtualGoalY", virtualGoal.getY());
        SmartDashboard.putNumber("SOTM/VirtualDistance", currentDistance);

        return new VirtualGoalResult(virtualGoal, currentDistance, virtualAngle, tof, true);
    }

    /**
     * Estimates ball time-of-flight based on flywheel RPM, hood angle, and distance.
     */
    private double estimateTimeOfFlight(double rpm, double hoodAngleDeg, double distanceMeters) {
        // Surface speed of flywheel
        double surfaceSpeedMps = rpm * Math.PI * FlywheelConstants.WHEEL_DIAMETER_METERS / 60.0;
        double exitVelocity = surfaceSpeedMps * FlywheelConstants.EXIT_VELOCITY_EFFICIENCY;

        // Horizontal component of exit velocity
        double hoodAngleRad = Math.toRadians(hoodAngleDeg);
        double horizontalVelocity = exitVelocity * Math.cos(hoodAngleRad);

        if (horizontalVelocity <= 0.0) {
            return ShootOnTheMoveConstants.MAX_TIME_OF_FLIGHT_SECONDS;
        }

        double tof = distanceMeters / horizontalVelocity;
        return Math.min(tof, ShootOnTheMoveConstants.MAX_TIME_OF_FLIGHT_SECONDS);
    }

    /**
     * Returns the best available flywheel lookup table (generated or fallback).
     */
    private double[][] getFlywheelTable() {
        return TurretLookupTables.FLYWHEEL_LOOKUP_TABLE.length > 0
                ? TurretLookupTables.FLYWHEEL_LOOKUP_TABLE
                : TurretAimingConstants.FLYWHEEL_LOOKUP_TABLE;
    }

    /**
     * Returns the best available hood lookup table (generated or fallback).
     */
    private double[][] getHoodTable() {
        return TurretLookupTables.HOOD_LOOKUP_TABLE.length > 0
                ? TurretLookupTables.HOOD_LOOKUP_TABLE
                : TurretAimingConstants.HOOD_LOOKUP_TABLE;
    }

    /**
     * Linear interpolation from a lookup table.
     *
     * @param input       The input value (e.g., distance)
     * @param lookupTable 2D array where [i][0] is input and [i][1] is output
     * @return Interpolated output value
     */
    private double interpolate(double input, double[][] lookupTable) {
        if (lookupTable.length == 0) {
            return 0.0;
        }

        if (input <= lookupTable[0][0]) {
            return lookupTable[0][1];
        }
        if (input >= lookupTable[lookupTable.length - 1][0]) {
            return lookupTable[lookupTable.length - 1][1];
        }

        for (int i = 0; i < lookupTable.length - 1; i++) {
            if (input >= lookupTable[i][0] && input <= lookupTable[i + 1][0]) {
                double t = (input - lookupTable[i][0]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                return lookupTable[i][1] + t * (lookupTable[i + 1][1] - lookupTable[i][1]);
            }
        }

        return 0.0;
    }
}
