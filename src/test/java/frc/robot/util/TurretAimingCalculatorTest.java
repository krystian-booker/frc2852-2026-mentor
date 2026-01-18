package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Method;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.TurretAimingConstants;
import frc.robot.util.TurretAimingCalculator.AimingResult;

/**
 * Unit tests for TurretAimingCalculator with 100% code coverage.
 */
class TurretAimingCalculatorTest {

    private TurretAimingCalculator calculator;

    @BeforeAll
    static void initializeHAL() {
        // Initialize HAL for WPILib classes (SmartDashboard, DriverStation, etc.)
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        // Default calculator with a fixed pose for most tests
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(8.0, 4.0, new Rotation2d(0)));
    }

    // ========== AimingResult Record Tests ==========

    @Test
    @DisplayName("AimingResult record stores values correctly")
    void aimingResultStoresValuesCorrectly() {
        AimingResult result = new AimingResult(45.0, 5.5, true);

        assertEquals(45.0, result.turretAngleDegrees(), 0.001);
        assertEquals(5.5, result.distanceMeters(), 0.001);
        assertTrue(result.isReachable());
    }

    @Test
    @DisplayName("AimingResult record handles unreachable state")
    void aimingResultHandlesUnreachable() {
        AimingResult result = new AimingResult(180.0, 15.0, false);

        assertEquals(180.0, result.turretAngleDegrees(), 0.001);
        assertEquals(15.0, result.distanceMeters(), 0.001);
        assertFalse(result.isReachable());
    }

    // ========== Constructor Tests ==========

    @Test
    @DisplayName("Constructor accepts pose supplier")
    void constructorAcceptsPoseSupplier() {
        Pose2d testPose = new Pose2d(1.0, 2.0, new Rotation2d(0));
        TurretAimingCalculator calc = new TurretAimingCalculator(() -> testPose);

        assertNotNull(calc);
        AimingResult result = calc.calculate();
        assertNotNull(result);
    }

    // ========== calculate() - Null/NaN Handling Tests ==========

    @Test
    @DisplayName("calculate() returns default result for null pose")
    void calculateReturnsDefaultForNullPose() {
        calculator = new TurretAimingCalculator(() -> null);

        AimingResult result = calculator.calculate();

        assertEquals(0.0, result.turretAngleDegrees(), 0.001);
        assertEquals(0.0, result.distanceMeters(), 0.001);
        assertFalse(result.isReachable());
    }

    @Test
    @DisplayName("calculate() returns default result for NaN X coordinate")
    void calculateReturnsDefaultForNaNX() {
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(Double.NaN, 4.0, new Rotation2d(0)));

        AimingResult result = calculator.calculate();

        assertEquals(0.0, result.turretAngleDegrees(), 0.001);
        assertEquals(0.0, result.distanceMeters(), 0.001);
        assertFalse(result.isReachable());
    }

    @Test
    @DisplayName("calculate() returns default result for NaN Y coordinate")
    void calculateReturnsDefaultForNaNY() {
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(8.0, Double.NaN, new Rotation2d(0)));

        AimingResult result = calculator.calculate();

        assertEquals(0.0, result.turretAngleDegrees(), 0.001);
        assertEquals(0.0, result.distanceMeters(), 0.001);
        assertFalse(result.isReachable());
    }

    // ========== calculate() - Angle Calculation Tests ==========

    @Test
    @DisplayName("calculate() computes correct angle when robot faces target")
    void calculateCorrectAngleWhenFacingTarget() {
        // Position robot directly to the right of blue target, facing left (toward target)
        // Blue target is at (0.0, 5.55)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(5.0, 5.55, Rotation2d.fromDegrees(180)));

        AimingResult result = calculator.calculate();

        // Robot is facing target, turret should point forward (0 degrees)
        assertEquals(0.0, result.turretAngleDegrees(), 1.0);
    }

    @Test
    @DisplayName("calculate() computes correct angle when target is behind robot")
    void calculateCorrectAngleWhenTargetBehind() {
        // Position robot to the left of blue target, facing left (away from target)
        // Blue target is at (0.0, 5.55)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(5.0, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        // Robot is facing away, turret should point backward (180 degrees)
        assertEquals(180.0, result.turretAngleDegrees(), 1.0);
    }

    @Test
    @DisplayName("calculate() computes correct angle when target is to the right")
    void calculateCorrectAngleWhenTargetRight() {
        // Blue target is at (0.0, 5.55)
        // Robot at (0.0, 0.0) facing up (90 deg), target to the left
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90)));

        AimingResult result = calculator.calculate();

        // Target is directly ahead and to the left, angle should be close to 0
        // Actually target at (0, 5.55), robot at (0,0) facing 90 degrees
        // Field angle to target is 90 deg, robot heading is 90 deg, so robot-relative is 0
        assertEquals(0.0, result.turretAngleDegrees(), 1.0);
    }

    @Test
    @DisplayName("calculate() normalizes negative angles to positive")
    void calculateNormalizesNegativeAngles() {
        // Set up a scenario that produces a negative angle before normalization
        // Robot facing right (0 degrees), target above and to the left
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        // Angle should be normalized to [0, 360)
        assertTrue(result.turretAngleDegrees() >= 0.0);
        assertTrue(result.turretAngleDegrees() < 360.0);
    }

    @Test
    @DisplayName("calculate() handles robot rotation correctly")
    void calculateHandlesRobotRotation() {
        // Robot at various rotations, check angle is computed relative to robot heading
        double[] testRotations = { 0, 45, 90, 135, 180, 225, 270, 315 };

        for (double rotation : testRotations) {
            calculator = new TurretAimingCalculator(
                    () -> new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(rotation)));

            AimingResult result = calculator.calculate();

            // All angles should be normalized
            assertTrue(result.turretAngleDegrees() >= 0.0,
                    "Angle should be >= 0 for rotation " + rotation);
            assertTrue(result.turretAngleDegrees() < 360.0,
                    "Angle should be < 360 for rotation " + rotation);
        }
    }

    // ========== calculate() - Distance Calculation Tests ==========

    @Test
    @DisplayName("calculate() computes correct distance to target")
    void calculateCorrectDistance() {
        // Blue target is at (0.0, 5.55)
        // Robot at (3.0, 5.55) - distance should be exactly 3.0 meters
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(3.0, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertEquals(3.0, result.distanceMeters(), 0.01);
    }

    @Test
    @DisplayName("calculate() computes correct diagonal distance")
    void calculateCorrectDiagonalDistance() {
        // Blue target is at (0.0, 5.55)
        // Robot at (3.0, 1.55) - distance should be 5.0 meters (3-4-5 triangle)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(3.0, 1.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertEquals(5.0, result.distanceMeters(), 0.01);
    }

    // ========== calculate() - Reachability Tests ==========

    @Test
    @DisplayName("calculate() marks target as reachable when in valid range")
    void calculateReachableWhenInRange() {
        // Blue target at (0.0, 5.55), valid range is 1.5 to 10.0 meters
        // Robot at (5.0, 5.55) - distance is 5.0 meters (in range)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(5.0, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertEquals(5.0, result.distanceMeters(), 0.01);
        assertTrue(result.isReachable());
    }

    @Test
    @DisplayName("calculate() marks target as unreachable when too close")
    void calculateUnreachableWhenTooClose() {
        // Robot at (1.0, 5.55) - distance is 1.0 meters (less than MIN 1.5)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(1.0, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertEquals(1.0, result.distanceMeters(), 0.01);
        assertFalse(result.isReachable());
    }

    @Test
    @DisplayName("calculate() marks target as unreachable when too far")
    void calculateUnreachableWhenTooFar() {
        // Robot at (12.0, 5.55) - distance is 12.0 meters (greater than MAX 10.0)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(12.0, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertEquals(12.0, result.distanceMeters(), 0.01);
        assertFalse(result.isReachable());
    }

    @Test
    @DisplayName("calculate() marks target as reachable at minimum distance boundary")
    void calculateReachableAtMinBoundary() {
        // Robot at exactly min distance from target
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS, 5.55,
                        Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertTrue(result.isReachable());
    }

    @Test
    @DisplayName("calculate() marks target as reachable at maximum distance boundary")
    void calculateReachableAtMaxBoundary() {
        // Robot at exactly max distance from target
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS, 5.55,
                        Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        assertTrue(result.isReachable());
    }

    // ========== getTargetPosition() Tests ==========

    @Test
    @DisplayName("getTargetPosition() returns a valid Translation2d")
    void getTargetPositionReturnsValidTranslation() {
        Translation2d target = calculator.getTargetPosition();

        assertNotNull(target);
        // Should be either blue or red target
        assertTrue(
                target.equals(TurretAimingConstants.BLUE_TARGET_POSITION)
                        || target.equals(TurretAimingConstants.RED_TARGET_POSITION));
    }

    @Test
    @DisplayName("getTargetPosition() returns blue target for Blue alliance")
    void getTargetPositionReturnsBlueTargetForBlueAlliance() {
        // Set alliance to Blue
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        Translation2d target = calculator.getTargetPosition();

        assertEquals(TurretAimingConstants.BLUE_TARGET_POSITION, target);
    }

    @Test
    @DisplayName("getTargetPosition() returns red target for Red alliance")
    void getTargetPositionReturnsRedTargetForRedAlliance() {
        // Set alliance to Red
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        Translation2d target = calculator.getTargetPosition();

        assertEquals(TurretAimingConstants.RED_TARGET_POSITION, target);

        // Reset to Blue for other tests
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    @Test
    @DisplayName("calculate() aims at Red target when on Red alliance")
    void calculateAimsAtRedTargetOnRedAlliance() {
        // Set alliance to Red
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        // Red target is at (16.54, 5.55)
        // Position robot at (11.54, 5.55) - 5 meters away from red target
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(11.54, 5.55, Rotation2d.fromDegrees(0)));

        AimingResult result = calculator.calculate();

        // Distance to red target should be 5 meters
        assertEquals(5.0, result.distanceMeters(), 0.01);
        // Robot facing right (0 deg), red target is to the right, turret should point forward
        assertEquals(0.0, result.turretAngleDegrees(), 1.0);
        assertTrue(result.isReachable());

        // Reset to Blue for other tests
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    // ========== getHoodAngle() - Interpolation Tests ==========

    @Test
    @DisplayName("getHoodAngle() returns correct value at table minimum")
    void getHoodAngleAtMinimum() {
        // Position robot so distance equals first lookup table entry (1.5 meters)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(1.5, 5.55, Rotation2d.fromDegrees(0)));

        double hoodAngle = calculator.getHoodAngle();

        // At 1.5m, hood should be 15.0 degrees
        assertEquals(15.0, hoodAngle, 0.1);
    }

    @Test
    @DisplayName("getHoodAngle() returns correct value at table maximum")
    void getHoodAngleAtMaximum() {
        // Position robot so distance equals last lookup table entry (10.0 meters)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(10.0, 5.55, Rotation2d.fromDegrees(0)));

        double hoodAngle = calculator.getHoodAngle();

        // At 10.0m, hood should be 45.0 degrees
        assertEquals(45.0, hoodAngle, 0.1);
    }

    @Test
    @DisplayName("getHoodAngle() interpolates between table entries")
    void getHoodAngleInterpolates() {
        // Position robot at 4.0m (between 3.0m and 5.0m entries)
        // At 3.0m -> 25.0 deg, at 5.0m -> 35.0 deg
        // At 4.0m -> should be 30.0 deg (midpoint)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(4.0, 5.55, Rotation2d.fromDegrees(0)));

        double hoodAngle = calculator.getHoodAngle();

        assertEquals(30.0, hoodAngle, 0.5);
    }

    @Test
    @DisplayName("getHoodAngle() clamps below minimum distance")
    void getHoodAngleClampsBelow() {
        // Position robot at 0.5m (below table minimum of 1.5m)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(0.5, 5.55, Rotation2d.fromDegrees(0)));

        double hoodAngle = calculator.getHoodAngle();

        // Should return first table value (15.0)
        assertEquals(15.0, hoodAngle, 0.1);
    }

    @Test
    @DisplayName("getHoodAngle() clamps above maximum distance")
    void getHoodAngleClampsAbove() {
        // Position robot at 15.0m (above table maximum of 10.0m)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(15.0, 5.55, Rotation2d.fromDegrees(0)));

        double hoodAngle = calculator.getHoodAngle();

        // Should return last table value (45.0)
        assertEquals(45.0, hoodAngle, 0.1);
    }

    // ========== getFlywheelRPM() - Interpolation Tests ==========

    @Test
    @DisplayName("getFlywheelRPM() returns correct value at table minimum")
    void getFlywheelRPMAtMinimum() {
        // Position robot so distance equals first lookup table entry (1.5 meters)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(1.5, 5.55, Rotation2d.fromDegrees(0)));

        double rpm = calculator.getFlywheelRPM();

        // At 1.5m, RPM should be 3000
        assertEquals(3000.0, rpm, 10.0);
    }

    @Test
    @DisplayName("getFlywheelRPM() returns correct value at table maximum")
    void getFlywheelRPMAtMaximum() {
        // Position robot so distance equals last lookup table entry (10.0 meters)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(10.0, 5.55, Rotation2d.fromDegrees(0)));

        double rpm = calculator.getFlywheelRPM();

        // At 10.0m, RPM should be 5000
        assertEquals(5000.0, rpm, 10.0);
    }

    @Test
    @DisplayName("getFlywheelRPM() interpolates between table entries")
    void getFlywheelRPMInterpolates() {
        // Position robot at 4.0m (between 3.0m and 5.0m entries)
        // At 3.0m -> 3500 RPM, at 5.0m -> 4000 RPM
        // At 4.0m -> should be 3750 RPM (midpoint)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(4.0, 5.55, Rotation2d.fromDegrees(0)));

        double rpm = calculator.getFlywheelRPM();

        assertEquals(3750.0, rpm, 25.0);
    }

    @Test
    @DisplayName("getFlywheelRPM() clamps below minimum distance")
    void getFlywheelRPMClampsBelow() {
        // Position robot at 0.5m (below table minimum of 1.5m)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(0.5, 5.55, Rotation2d.fromDegrees(0)));

        double rpm = calculator.getFlywheelRPM();

        // Should return first table value (3000)
        assertEquals(3000.0, rpm, 10.0);
    }

    @Test
    @DisplayName("getFlywheelRPM() clamps above maximum distance")
    void getFlywheelRPMClampsAbove() {
        // Position robot at 15.0m (above table maximum of 10.0m)
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(15.0, 5.55, Rotation2d.fromDegrees(0)));

        double rpm = calculator.getFlywheelRPM();

        // Should return last table value (5000)
        assertEquals(5000.0, rpm, 10.0);
    }

    // ========== interpolate() Edge Case Tests (via reflection) ==========

    @Test
    @DisplayName("interpolate() returns 0 for empty lookup table")
    void interpolateReturnsZeroForEmptyTable() throws Exception {
        // Use reflection to test the private interpolate method
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        double[][] emptyTable = {};
        double result = (double) interpolateMethod.invoke(calculator, 5.0, emptyTable);

        assertEquals(0.0, result, 0.001);
    }

    @Test
    @DisplayName("interpolate() handles single entry table")
    void interpolateHandlesSingleEntryTable() throws Exception {
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        double[][] singleTable = { { 5.0, 100.0 } };

        // Below single entry
        double resultBelow = (double) interpolateMethod.invoke(calculator, 3.0, singleTable);
        assertEquals(100.0, resultBelow, 0.001);

        // At single entry
        double resultAt = (double) interpolateMethod.invoke(calculator, 5.0, singleTable);
        assertEquals(100.0, resultAt, 0.001);

        // Above single entry
        double resultAbove = (double) interpolateMethod.invoke(calculator, 7.0, singleTable);
        assertEquals(100.0, resultAbove, 0.001);
    }

    @Test
    @DisplayName("interpolate() handles exact match on table entry")
    void interpolateHandlesExactMatch() throws Exception {
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        double[][] table = { { 1.0, 10.0 }, { 2.0, 20.0 }, { 3.0, 30.0 } };

        double result = (double) interpolateMethod.invoke(calculator, 2.0, table);
        assertEquals(20.0, result, 0.001);
    }

    @Test
    @DisplayName("interpolate() handles interpolation at various points")
    void interpolateHandlesVariousPoints() throws Exception {
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        double[][] table = { { 0.0, 0.0 }, { 10.0, 100.0 } };

        // 25% through
        double result25 = (double) interpolateMethod.invoke(calculator, 2.5, table);
        assertEquals(25.0, result25, 0.001);

        // 50% through
        double result50 = (double) interpolateMethod.invoke(calculator, 5.0, table);
        assertEquals(50.0, result50, 0.001);

        // 75% through
        double result75 = (double) interpolateMethod.invoke(calculator, 7.5, table);
        assertEquals(75.0, result75, 0.001);
    }

    @Test
    @DisplayName("interpolate() handles decreasing (malformed) table gracefully")
    void interpolateHandlesMalformedDecreasingTable() throws Exception {
        // Test that a malformed (decreasing) table still returns a value via clamp checks
        // Note: The final 'return 0.0' in interpolate() is unreachable defensive code
        // because for any input:
        // - If input <= first entry: returns first value
        // - If input >= last entry: returns last value
        // - Otherwise: loop always finds a matching interval for properly ordered tables
        // For decreasing tables, the clamp checks still catch all inputs
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        // Table with decreasing input values
        double[][] decreasingTable = { { 10.0, 100.0 }, { 5.0, 50.0 }, { 1.0, 10.0 } };

        // Input 7.0 is caught by first clamp check (7.0 <= 10.0)
        double result = (double) interpolateMethod.invoke(calculator, 7.0, decreasingTable);
        assertEquals(100.0, result, 0.001); // Returns first entry's value
    }

    @Test
    @DisplayName("interpolate() handles table with gap between entries")
    void interpolateHandlesGapBetweenEntries() throws Exception {
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        // Table with entries at 1 and 3
        double[][] gapTable = { { 1.0, 10.0 }, { 3.0, 30.0 } };

        // 2.0 is between 1.0 and 3.0, so interpolation finds it
        double result = (double) interpolateMethod.invoke(calculator, 2.0, gapTable);
        assertEquals(20.0, result, 0.001); // Linear interpolation
    }

    @Test
    @DisplayName("interpolate() handles three-element table")
    void interpolateHandlesThreeElementTable() throws Exception {
        Method interpolateMethod = TurretAimingCalculator.class.getDeclaredMethod(
                "interpolate", double.class, double[][].class);
        interpolateMethod.setAccessible(true);

        double[][] table = { { 1.0, 10.0 }, { 5.0, 50.0 }, { 10.0, 100.0 } };

        // Test interpolation in first segment
        double result1 = (double) interpolateMethod.invoke(calculator, 3.0, table);
        assertEquals(30.0, result1, 0.001);

        // Test interpolation in second segment
        double result2 = (double) interpolateMethod.invoke(calculator, 7.5, table);
        assertEquals(75.0, result2, 0.001);
    }

    // ========== Integration Tests ==========

    @Test
    @DisplayName("Full aiming calculation workflow")
    void fullAimingCalculationWorkflow() {
        // Robot at a realistic position
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(45)));

        AimingResult result = calculator.calculate();
        double hoodAngle = calculator.getHoodAngle();
        double flywheelRPM = calculator.getFlywheelRPM();

        // Verify all values are reasonable
        assertTrue(result.turretAngleDegrees() >= 0 && result.turretAngleDegrees() < 360);
        assertTrue(result.distanceMeters() > 0);
        assertTrue(hoodAngle >= 15.0 && hoodAngle <= 45.0);
        assertTrue(flywheelRPM >= 3000.0 && flywheelRPM <= 5000.0);
    }

    @Test
    @DisplayName("Repeated calculations return consistent results")
    void repeatedCalculationsConsistent() {
        calculator = new TurretAimingCalculator(
                () -> new Pose2d(6.0, 3.0, Rotation2d.fromDegrees(30)));

        AimingResult result1 = calculator.calculate();
        AimingResult result2 = calculator.calculate();
        AimingResult result3 = calculator.calculate();

        assertEquals(result1.turretAngleDegrees(), result2.turretAngleDegrees(), 0.001);
        assertEquals(result2.turretAngleDegrees(), result3.turretAngleDegrees(), 0.001);
        assertEquals(result1.distanceMeters(), result2.distanceMeters(), 0.001);
        assertEquals(result1.isReachable(), result2.isReachable());
    }
}
