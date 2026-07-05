package frc.robot.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterModelConstants;

/**
 * Distance-indexed shot model for a fixed-height target.
 *
 * <p>
 * Replaces the old 578-cell field-position lookup grid with a handful of
 * distance-keyed calibration points. Because the turret always points at the
 * target, the required hood angle and flywheel RPM depend only on horizontal
 * distance to the target, not on where the robot is on the field — so ~6
 * points across the shooting range fully describe the shot.
 *
 * <p>
 * Hood angle and flywheel RPM are linearly interpolated between calibration
 * points. Time of flight is derived from projectile geometry: a ball launched
 * at elevation θ that passes through a point at horizontal distance d and
 * height rise Δh satisfies Δh = d·tanθ − ½gt², independent of exit speed:
 *
 * <pre>t = sqrt(2 · (d·tanθ − Δh) / g)</pre>
 *
 * so every calibration point automatically yields the flight time the SOTM
 * solver needs. Aerodynamic drag on the foam ball makes real flight slightly
 * longer than this vacuum estimate; the solver applies a dashboard-tunable
 * scale factor on top.
 *
 * <p>
 * Points are persisted to a CSV file that survives code deploys, and the map
 * hot-reloads whenever a point is added or removed.
 */
public class ShotMap {

    /** One calibration point: at this distance, this hood/RPM lands the shot. */
    public record ShotPoint(double distanceMeters, double hoodDegrees, double flywheelRPM) {
    }

    private final String name;
    private final double targetHeightMeters;
    private final List<ShotPoint> defaultPoints;
    private final Path saveFile;

    // Insertion-ordered so removeLastPoint() can undo the most recent record
    private final List<ShotPoint> points = new ArrayList<>();

    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    /**
     * @param name               Human-readable name ("hub", "lob") used in
     *                           telemetry and log messages
     * @param targetHeightMeters Height of the target above the carpet (hub rim
     *                           height for scoring, 0 for lob landing)
     * @param defaultPoints      Points used when no saved calibration file exists
     * @param saveFile           File the calibration is persisted to (survives
     *                           deploys when outside the deploy directory)
     */
    public ShotMap(String name, double targetHeightMeters, List<ShotPoint> defaultPoints, Path saveFile) {
        this.name = name;
        this.targetHeightMeters = targetHeightMeters;
        this.defaultPoints = List.copyOf(defaultPoints);
        this.saveFile = saveFile;
        if (!loadFromFile()) {
            points.addAll(defaultPoints);
        }
        rebuildInterpolators();
    }

    /** Interpolated hood mechanism angle (degrees) for a horizontal distance. */
    public synchronized double getHoodDegrees(double distanceMeters) {
        if (points.isEmpty()) {
            return ShooterModelConstants.EMPTY_MAP_HOOD_DEGREES;
        }
        return hoodMap.get(distanceMeters);
    }

    /** Interpolated flywheel RPM for a horizontal distance. */
    public synchronized double getFlywheelRPM(double distanceMeters) {
        if (points.isEmpty()) {
            return ShooterModelConstants.EMPTY_MAP_FLYWHEEL_RPM;
        }
        return rpmMap.get(distanceMeters);
    }

    /**
     * Estimated ball flight time (seconds) for a horizontal distance, derived
     * from the calibrated arc geometry (vacuum model — the SOTM solver applies
     * a tunable drag scale on top).
     */
    public synchronized double getTimeOfFlightSeconds(double distanceMeters) {
        if (points.isEmpty()) {
            return distanceMeters / ShooterModelConstants.FALLBACK_BALL_SPEED_MPS;
        }
        return tofMap.get(distanceMeters);
    }

    /**
     * Records a calibration point. A previous point within
     * {@link ShooterModelConstants#POINT_MERGE_DISTANCE_METERS} is replaced so
     * re-shooting the same spot updates rather than duplicates. Persists to
     * disk and rebuilds the interpolators.
     */
    public synchronized void addPoint(double distanceMeters, double hoodDegrees, double flywheelRPM) {
        points.removeIf(p -> Math.abs(p.distanceMeters() - distanceMeters) //
                < ShooterModelConstants.POINT_MERGE_DISTANCE_METERS);
        points.add(new ShotPoint(distanceMeters, hoodDegrees, flywheelRPM));
        rebuildInterpolators();
        saveToFile();
    }

    /**
     * Removes the most recently added point.
     *
     * @return true if a point was removed
     */
    public synchronized boolean removeLastPoint() {
        if (points.isEmpty()) {
            return false;
        }
        points.remove(points.size() - 1);
        rebuildInterpolators();
        saveToFile();
        return true;
    }

    /** Discards the saved calibration and restores the in-code defaults. */
    public synchronized void resetToDefaults() {
        points.clear();
        points.addAll(defaultPoints);
        rebuildInterpolators();
        try {
            Files.deleteIfExists(saveFile);
        } catch (IOException e) {
            DriverStation.reportError("ShotMap[" + name + "] failed to delete " + saveFile + ": " + e, false);
        }
    }

    public synchronized int getPointCount() {
        return points.size();
    }

    public String getName() {
        return name;
    }

    /**
     * CSV dump of the current points (sorted by distance), suitable for
     * pasting into the in-code defaults after a good calibration session.
     */
    public synchronized String toCsv() {
        StringBuilder sb = new StringBuilder("# distance_m,hood_deg,flywheel_rpm\n");
        points.stream()
                .sorted(Comparator.comparingDouble(ShotPoint::distanceMeters))
                .forEach(p -> sb.append(String.format("%.3f,%.2f,%.0f%n",
                        p.distanceMeters(), p.hoodDegrees(), p.flywheelRPM())));
        return sb.toString();
    }

    private void rebuildInterpolators() {
        hoodMap.clear();
        rpmMap.clear();
        tofMap.clear();
        for (ShotPoint p : points) {
            hoodMap.put(p.distanceMeters(), p.hoodDegrees());
            rpmMap.put(p.distanceMeters(), p.flywheelRPM());
            tofMap.put(p.distanceMeters(), ballisticTimeOfFlight(p.distanceMeters(), p.hoodDegrees()));
        }
    }

    /**
     * Vacuum-model flight time to a point at {@code targetHeightMeters}, given
     * the calibrated hood position at that distance. Falls back to a constant
     * average ball speed if the geometry is degenerate (target above the reach
     * of the launch angle).
     */
    private double ballisticTimeOfFlight(double distanceMeters, double hoodDegrees) {
        double launchRadians = Math.toRadians(HoodConstants.mechanismToActualAngle(hoodDegrees));
        double rise = targetHeightMeters - ShooterModelConstants.SHOOTER_EXIT_HEIGHT_METERS;
        double tSquared = 2.0 * (distanceMeters * Math.tan(launchRadians) - rise) / ShooterModelConstants.GRAVITY;
        if (tSquared <= 1e-6) {
            return distanceMeters / ShooterModelConstants.FALLBACK_BALL_SPEED_MPS;
        }
        return Math.sqrt(tSquared);
    }

    private boolean loadFromFile() {
        if (!Files.exists(saveFile)) {
            return false;
        }
        try {
            List<ShotPoint> loaded = new ArrayList<>();
            for (String line : Files.readAllLines(saveFile)) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                String[] parts = line.split(",");
                loaded.add(new ShotPoint(
                        Double.parseDouble(parts[0].trim()),
                        Double.parseDouble(parts[1].trim()),
                        Double.parseDouble(parts[2].trim())));
            }
            if (loaded.isEmpty()) {
                return false;
            }
            points.clear();
            points.addAll(loaded);
            System.out.println("ShotMap[" + name + "] loaded " + loaded.size() + " points from " + saveFile);
            return true;
        } catch (IOException | RuntimeException e) {
            DriverStation.reportError("ShotMap[" + name + "] failed to load " + saveFile
                    + " — using defaults: " + e, false);
            return false;
        }
    }

    private void saveToFile() {
        try {
            Files.createDirectories(saveFile.getParent());
            Files.writeString(saveFile, toCsv());
            System.out.println("ShotMap[" + name + "] saved " + points.size() + " points to " + saveFile);
        } catch (IOException e) {
            DriverStation.reportError("ShotMap[" + name + "] failed to save " + saveFile + ": " + e, false);
        }
    }
}
