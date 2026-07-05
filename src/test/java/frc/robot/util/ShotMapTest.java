package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterModelConstants;
import frc.robot.util.ShotMap.ShotPoint;

class ShotMapTest {

    private static final double HUB_HEIGHT = ShooterModelConstants.HUB_RIM_HEIGHT_METERS;

    private static final List<ShotPoint> TWO_POINTS = List.of(
            new ShotPoint(2.0, 10.0, 2500),
            new ShotPoint(4.0, 20.0, 3000));

    @TempDir
    Path tempDir;

    private ShotMap newMap(List<ShotPoint> defaults) {
        return new ShotMap("test", HUB_HEIGHT, defaults, tempDir.resolve("test.csv"));
    }

    @Test
    void interpolatesBetweenPoints() {
        ShotMap map = newMap(TWO_POINTS);
        assertEquals(15.0, map.getHoodDegrees(3.0), 1e-9);
        assertEquals(2750.0, map.getFlywheelRPM(3.0), 1e-9);
    }

    @Test
    void clampsOutsideCalibratedRange() {
        ShotMap map = newMap(TWO_POINTS);
        assertEquals(10.0, map.getHoodDegrees(0.5), 1e-9);
        assertEquals(20.0, map.getHoodDegrees(9.0), 1e-9);
        assertEquals(3000.0, map.getFlywheelRPM(9.0), 1e-9);
    }

    @Test
    void timeOfFlightMatchesBallisticGeometry() {
        // A ball launched at elevation theta passing through (d, rise)
        // satisfies rise = d*tan(theta) - g*t^2/2, independent of exit speed
        ShotMap map = newMap(List.of(new ShotPoint(3.0, 15.0, 2750)));
        double launchRad = Math.toRadians(HoodConstants.mechanismToActualAngle(15.0));
        double rise = HUB_HEIGHT - ShooterModelConstants.SHOOTER_EXIT_HEIGHT_METERS;
        double expected = Math.sqrt(2.0 * (3.0 * Math.tan(launchRad) - rise) / ShooterModelConstants.GRAVITY);
        assertEquals(expected, map.getTimeOfFlightSeconds(3.0), 1e-9);
        assertTrue(expected > 0.3 && expected < 2.0, "ToF should be physically plausible");
    }

    @Test
    void timeOfFlightIncreasesWithDistanceOnRealCurve() {
        ShotMap map = newMap(toPoints(ShooterModelConstants.HUB_DEFAULT_POINTS));
        double previous = 0;
        for (double d = 1.5; d <= 6.0; d += 0.5) {
            double tof = map.getTimeOfFlightSeconds(d);
            assertTrue(tof > 0, "ToF must be positive at " + d);
            assertTrue(tof >= previous - 0.15, "ToF should not collapse with distance at " + d);
            previous = tof;
        }
    }

    @Test
    void addPointPersistsAndReloads() {
        ShotMap map = newMap(TWO_POINTS);
        map.addPoint(3.0, 14.0, 2800);
        assertEquals(3, map.getPointCount());
        assertTrue(Files.exists(tempDir.resolve("test.csv")));

        // A new instance with different defaults must load the saved points
        ShotMap reloaded = newMap(List.of());
        assertEquals(3, reloaded.getPointCount());
        assertEquals(14.0, reloaded.getHoodDegrees(3.0), 1e-9);
    }

    @Test
    void addPointReplacesNearbyPoint() {
        ShotMap map = newMap(TWO_POINTS);
        map.addPoint(2.05, 11.0, 2550); // within merge distance of the 2.0 point
        assertEquals(2, map.getPointCount());
        assertEquals(11.0, map.getHoodDegrees(2.0), 1e-9);
    }

    @Test
    void undoRemovesLastRecordedPoint() {
        ShotMap map = newMap(TWO_POINTS);
        map.addPoint(3.0, 14.0, 2800);
        assertTrue(map.removeLastPoint());
        assertEquals(2, map.getPointCount());
        assertEquals(15.0, map.getHoodDegrees(3.0), 1e-9); // back to interpolated
    }

    @Test
    void resetRestoresDefaultsAndDeletesFile() {
        ShotMap map = newMap(TWO_POINTS);
        map.addPoint(3.0, 14.0, 2800);
        map.resetToDefaults();
        assertEquals(2, map.getPointCount());
        assertFalse(Files.exists(tempDir.resolve("test.csv")));
    }

    @Test
    void emptyMapFallsBackToSafeValues() {
        ShotMap map = newMap(List.of());
        assertEquals(ShooterModelConstants.EMPTY_MAP_HOOD_DEGREES, map.getHoodDegrees(3.0), 1e-9);
        assertEquals(ShooterModelConstants.EMPTY_MAP_FLYWHEEL_RPM, map.getFlywheelRPM(3.0), 1e-9);
        assertEquals(3.0 / ShooterModelConstants.FALLBACK_BALL_SPEED_MPS, map.getTimeOfFlightSeconds(3.0), 1e-9);
    }

    private static List<ShotPoint> toPoints(double[][] raw) {
        return java.util.Arrays.stream(raw)
                .map(p -> new ShotPoint(p[0], p[1], p[2]))
                .toList();
    }
}
