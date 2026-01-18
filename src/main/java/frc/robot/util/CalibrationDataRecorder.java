package frc.robot.util;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

import frc.robot.Constants.CalibrationConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretAimingConstants;

/**
 * Manages calibration data recording and CSV file I/O on the roboRIO.
 * Stores calibration points in memory and writes to CSV file on demand.
 *
 * <p>CSV Format: timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,
 * flywheel_rpm,grid_row,grid_col,alliance
 *
 * <p>Note: CSV parsing uses simple comma splitting (String.split(",")) which assumes
 * no fields contain embedded commas. This is safe for the current format since:
 * - Timestamps use ISO 8601 format (no commas)
 * - All numeric fields are simple decimals
 * - Alliance is either "Blue", "Red", or "Unknown"
 * If future fields require embedded commas, a proper CSV parser library should be used.
 */
public class CalibrationDataRecorder {

    /**
     * Represents a single calibration data point.
     */
    public record CalibrationPoint(
            String timestamp,
            double robotX,
            double robotY,
            double distanceToTarget,
            double hoodAngleDegrees,
            double flywheelRPM,
            int gridRow,
            int gridCol,
            String alliance) {

        /**
         * Converts the calibration point to a CSV row string.
         */
        public String toCsvRow() {
            return String.format("%s,%.3f,%.3f,%.3f,%.2f,%.0f,%d,%d,%s",
                    timestamp, robotX, robotY, distanceToTarget,
                    hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);
        }
    }

    private static final String CSV_HEADER = "timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,flywheel_rpm,grid_row,grid_col,alliance";
    private static final DateTimeFormatter TIMESTAMP_FORMAT = DateTimeFormatter.ISO_LOCAL_DATE_TIME;

    private final List<CalibrationPoint> points;
    private final String filePath;
    private final StringPublisher errorPub;
    private final StringArrayPublisher pointsPub;
    private final IntegerPublisher versionPub;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private int dataVersion = 0;

    /**
     * Creates a new CalibrationDataRecorder with the default file path.
     */
    public CalibrationDataRecorder() {
        this(CalibrationConstants.CALIBRATION_FILE_PATH);
    }

    /**
     * Creates a new CalibrationDataRecorder with a custom file path.
     *
     * @param filePath The path to the CSV file
     */
    public CalibrationDataRecorder(String filePath) {
        this.filePath = filePath;
        this.points = new ArrayList<>();
        var table = NetworkTableInstance.getDefault().getTable("TurretCalibration");
        this.errorPub = table.getStringTopic("Error").publish();
        this.pointsPub = table.getSubTable("Data").getStringArrayTopic("Points").publish();
        this.versionPub = table.getSubTable("Data").getIntegerTopic("Version").publish();
        ensureDirectoryExists();
    }

    /**
     * Ensures the parent directory for the calibration file exists.
     */
    private void ensureDirectoryExists() {
        File file = new File(filePath);
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            if (!parentDir.mkdirs() && !parentDir.exists()) {
                String errorMsg = "Failed to create calibration directory: " + parentDir.getAbsolutePath();
                System.err.println(errorMsg);
                errorPub.set(errorMsg);
            }
        }
    }

    /**
     * Loads calibration data from the CSV file into memory.
     * This allows resuming a calibration session after a robot reboot.
     *
     * @return true if data was loaded successfully (or file doesn't exist), false on error
     */
    public boolean loadFromFile() {
        File file = new File(filePath);
        if (!file.exists()) {
            // No file to load - this is not an error
            return true;
        }

        lock.writeLock().lock();
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            boolean headerSkipped = false;
            int loadedCount = 0;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) {
                    continue;
                }

                // Skip header row - handle BOM and case-insensitivity
                if (!headerSkipped) {
                    // Remove UTF-8 BOM if present (common in Excel-exported CSVs)
                    if (line.startsWith("\uFEFF")) {
                        line = line.substring(1);
                    }
                    String lowerLine = line.toLowerCase().trim();
                    boolean isHeader = lowerLine.startsWith("timestamp") ||
                            lowerLine.contains("robot_x") ||
                            lowerLine.contains("hood_angle");
                    headerSkipped = true;
                    if (isHeader) {
                        continue;
                    }
                }

                // Parse CSV line: timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,flywheel_rpm,grid_row,grid_col,alliance
                String[] parts = line.split(",");
                if (parts.length >= 8) {
                    try {
                        String timestamp = parts[0].trim();
                        double robotX = Double.parseDouble(parts[1].trim());
                        double robotY = Double.parseDouble(parts[2].trim());
                        double distanceToTarget = Double.parseDouble(parts[3].trim());
                        double hoodAngleDegrees = Double.parseDouble(parts[4].trim());
                        double flywheelRPM = Double.parseDouble(parts[5].trim());
                        int gridRow = Integer.parseInt(parts[6].trim());
                        int gridCol = Integer.parseInt(parts[7].trim());

                        // Alliance is optional (for backward compatibility)
                        String alliance = "Unknown";
                        if (parts.length >= 9 && !parts[8].trim().isEmpty()) {
                            alliance = parts[8].trim();
                        }

                        // Validate data before adding (check against mechanical limits and grid bounds)
                        if (distanceToTarget >= TurretAimingConstants.MIN_SHOOTING_DISTANCE_METERS
                                && distanceToTarget <= TurretAimingConstants.MAX_SHOOTING_DISTANCE_METERS
                                && hoodAngleDegrees >= HoodConstants.MIN_POSITION_DEGREES
                                && hoodAngleDegrees <= HoodConstants.MAX_POSITION_DEGREES
                                && flywheelRPM >= FlywheelConstants.MIN_RPM
                                && flywheelRPM <= FlywheelConstants.MAX_RPM
                                && gridRow >= 0 && gridRow < CalibrationConstants.GRID_ROWS
                                && gridCol >= 0 && gridCol < CalibrationConstants.GRID_COLS) {
                            CalibrationPoint point = new CalibrationPoint(
                                    timestamp, robotX, robotY, distanceToTarget,
                                    hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);
                            points.add(point);
                            loadedCount++;
                        } else {
                            System.err.println("Warning: Skipping point with invalid data - distance: "
                                    + distanceToTarget + ", hood: " + hoodAngleDegrees
                                    + ", RPM: " + flywheelRPM + ", grid: [" + gridRow + "," + gridCol + "]");
                        }
                    } catch (NumberFormatException e) {
                        System.err.println("Warning: Skipping malformed calibration line: " + line);
                    }
                }
            }

            System.out.println("Loaded " + loadedCount + " calibration points from " + filePath);
            errorPub.set(""); // Clear any previous error

        } catch (IOException e) {
            String errorMsg = "Failed to load calibration data: " + e.getMessage();
            System.err.println(errorMsg);
            errorPub.set(errorMsg);
            return false;
        } finally {
            lock.writeLock().unlock();
        }

        // Publish to NetworkTables after releasing write lock
        publishPointsToNetworkTables();
        return true;
    }

    // Field dimensions for position validation (use constants from CalibrationConstants)
    private static final double FIELD_LENGTH_METERS = CalibrationConstants.FIELD_LENGTH_METERS;
    private static final double FIELD_WIDTH_METERS = CalibrationConstants.FIELD_WIDTH_METERS;

    /**
     * Adds a calibration point to the in-memory buffer.
     *
     * @param robotX           Robot X position in meters
     * @param robotY           Robot Y position in meters
     * @param distanceToTarget Distance to target in meters
     * @param hoodAngleDegrees Hood angle in degrees
     * @param flywheelRPM      Flywheel speed in RPM
     * @param gridRow          Grid row index
     * @param gridCol          Grid column index
     * @param alliance         Alliance color ("Blue" or "Red")
     * @throws IllegalArgumentException if any values are invalid (NaN, Infinity, or out of range)
     */
    public void addPoint(double robotX, double robotY, double distanceToTarget,
            double hoodAngleDegrees, double flywheelRPM, int gridRow, int gridCol, String alliance) {
        // Validate inputs
        if (Double.isNaN(distanceToTarget) || Double.isInfinite(distanceToTarget) || distanceToTarget <= 0) {
            throw new IllegalArgumentException("Invalid distance: " + distanceToTarget);
        }
        if (Double.isNaN(hoodAngleDegrees) || Double.isInfinite(hoodAngleDegrees)) {
            throw new IllegalArgumentException("Invalid hood angle: " + hoodAngleDegrees);
        }
        if (Double.isNaN(flywheelRPM) || Double.isInfinite(flywheelRPM) || flywheelRPM <= 0) {
            throw new IllegalArgumentException("Invalid flywheel RPM: " + flywheelRPM);
        }
        if (Double.isNaN(robotX) || Double.isInfinite(robotX) ||
            Double.isNaN(robotY) || Double.isInfinite(robotY)) {
            throw new IllegalArgumentException("Invalid robot position: (" + robotX + ", " + robotY + ")");
        }
        // Validate robot position is within field boundaries
        if (robotX < 0 || robotX > FIELD_LENGTH_METERS) {
            throw new IllegalArgumentException("Robot X position " + robotX + " outside field bounds (0-" + FIELD_LENGTH_METERS + "m)");
        }
        if (robotY < 0 || robotY > FIELD_WIDTH_METERS) {
            throw new IllegalArgumentException("Robot Y position " + robotY + " outside field bounds (0-" + FIELD_WIDTH_METERS + "m)");
        }
        if (gridRow < 0 || gridRow >= CalibrationConstants.GRID_ROWS) {
            throw new IllegalArgumentException("Invalid grid row: " + gridRow + " (expected 0-" + (CalibrationConstants.GRID_ROWS - 1) + ")");
        }
        if (gridCol < 0 || gridCol >= CalibrationConstants.GRID_COLS) {
            throw new IllegalArgumentException("Invalid grid col: " + gridCol + " (expected 0-" + (CalibrationConstants.GRID_COLS - 1) + ")");
        }

        String timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT);
        CalibrationPoint point = new CalibrationPoint(
                timestamp, robotX, robotY, distanceToTarget,
                hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);

        lock.writeLock().lock();
        try {
            points.add(point);
        } finally {
            lock.writeLock().unlock();
        }

        // Publish to NetworkTables after releasing lock
        publishPointsToNetworkTables();
    }

    /**
     * Adds a calibration point to the in-memory buffer with unknown alliance.
     * Provided for backward compatibility.
     *
     * @param robotX           Robot X position in meters
     * @param robotY           Robot Y position in meters
     * @param distanceToTarget Distance to target in meters
     * @param hoodAngleDegrees Hood angle in degrees
     * @param flywheelRPM      Flywheel speed in RPM
     * @param gridRow          Grid row index
     * @param gridCol          Grid column index
     */
    public void addPoint(double robotX, double robotY, double distanceToTarget,
            double hoodAngleDegrees, double flywheelRPM, int gridRow, int gridCol) {
        addPoint(robotX, robotY, distanceToTarget, hoodAngleDegrees, flywheelRPM, gridRow, gridCol, "Unknown");
    }

    /**
     * Adds a pre-constructed calibration point to the buffer.
     *
     * @param point The calibration point to add
     */
    public void addPoint(CalibrationPoint point) {
        lock.writeLock().lock();
        try {
            points.add(point);
        } finally {
            lock.writeLock().unlock();
        }

        // Publish to NetworkTables after releasing lock
        publishPointsToNetworkTables();
    }

    /**
     * Writes all buffered points to the CSV file.
     * Creates a backup of existing file before overwriting.
     *
     * <p>Uses atomic write pattern (temp-file-then-rename) to prevent corruption
     * on partial writes. File I/O is performed outside the lock to improve concurrency.
     * Points are copied while holding a read lock, then file operations
     * proceed without blocking other threads.
     *
     * @return true if the file was written successfully, false otherwise
     */
    public boolean saveToFile() {
        // Copy points while holding read lock (fast operation)
        List<CalibrationPoint> pointsCopy;
        lock.readLock().lock();
        try {
            pointsCopy = new ArrayList<>(points);
        } finally {
            lock.readLock().unlock();
        }

        // File I/O operations without holding lock (potentially slow)
        File targetFile = new File(filePath);
        File tempFile = new File(filePath + ".tmp");

        // Create backup of existing file before overwriting
        // If backup fails and there's existing data, abort to prevent data loss
        boolean backupRequired = false;
        try {
            backupRequired = targetFile.exists() && targetFile.length() > 0;
            if (backupRequired) {
                String backupPath = filePath.replace(".csv",
                        "_backup_" + LocalDateTime.now().format(
                                DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")) + ".csv");
                try {
                    Files.copy(targetFile.toPath(), Paths.get(backupPath));
                } catch (IOException e) {
                    String errorMsg = "CRITICAL: Failed to create backup before save: " + e.getMessage() +
                            ". Aborting save to prevent data loss.";
                    System.err.println(errorMsg);
                    errorPub.set(errorMsg);
                    return false; // Don't overwrite if backup fails
                }
            }
        } catch (Exception e) {
            if (backupRequired) {
                String errorMsg = "CRITICAL: Backup creation failed: " + e.getMessage() +
                        ". Aborting save to prevent data loss.";
                System.err.println(errorMsg);
                errorPub.set(errorMsg);
                return false; // Don't overwrite if backup fails
            }
        }

        // Write to temp file first (atomic write pattern)
        try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(tempFile, false)))) {
            writer.println(CSV_HEADER);
            for (CalibrationPoint point : pointsCopy) {
                writer.println(point.toCsvRow());
            }
            writer.flush();
        } catch (IOException e) {
            String errorMsg = "Failed to write calibration data to temp file: " + e.getMessage();
            System.err.println(errorMsg);
            errorPub.set(errorMsg);
            // Clean up temp file on failure
            try {
                Files.deleteIfExists(tempFile.toPath());
            } catch (IOException ignored) {
            }
            return false;
        }

        // Atomically rename temp file to target file
        try {
            // Delete target if it exists (required on Windows for atomic rename)
            Files.deleteIfExists(targetFile.toPath());
            Files.move(tempFile.toPath(), targetFile.toPath());
            errorPub.set(""); // Clear any previous error
            return true;
        } catch (IOException e) {
            String errorMsg = "Failed to finalize calibration file: " + e.getMessage();
            System.err.println(errorMsg);
            errorPub.set(errorMsg);
            // Clean up temp file on failure
            try {
                Files.deleteIfExists(tempFile.toPath());
            } catch (IOException ignored) {
            }
            return false;
        }
    }

    /**
     * Clears all buffered points and deletes the CSV file if it exists.
     */
    public void clearData() {
        lock.writeLock().lock();
        try {
            points.clear();
        } finally {
            lock.writeLock().unlock();
        }
        // Use atomic file deletion to avoid TOCTOU race condition
        try {
            Files.deleteIfExists(Paths.get(filePath));
        } catch (IOException e) {
            String errorMsg = "Failed to delete calibration file: " + e.getMessage();
            System.err.println(errorMsg);
            errorPub.set(errorMsg);
        }

        // Publish to NetworkTables after clearing
        publishPointsToNetworkTables();
    }

    /**
     * Gets the number of points currently in the buffer.
     *
     * @return Number of saved points
     */
    public int getPointCount() {
        lock.readLock().lock();
        try {
            return points.size();
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Gets the total number of grid cells (for progress tracking).
     *
     * @return Total grid cells
     */
    public int getTotalGridCells() {
        return CalibrationConstants.GRID_COLS * CalibrationConstants.GRID_ROWS;
    }

    /**
     * Finalizes the calibration session by saving and marking complete.
     *
     * @return true if export was successful
     */
    public boolean exportFinal() {
        return saveToFile();
    }

    /**
     * Gets all calibration points (read-only copy).
     *
     * @return List of all calibration points
     */
    public List<CalibrationPoint> getPoints() {
        lock.readLock().lock();
        try {
            return new ArrayList<>(points);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Gets the file path being used for calibration data.
     *
     * @return The file path
     */
    public String getFilePath() {
        return filePath;
    }

    /**
     * Publishes all calibration points to NetworkTables for webapp synchronization.
     * Serializes each point as a CSV row and publishes the array along with a version number.
     * The version number increments on each publish, allowing clients to detect changes.
     *
     * <p>Uses write lock because dataVersion is modified. The serialization, version increment,
     * and publishing are all done inside the lock to ensure atomic consistency between version
     * and data. This prevents race conditions where a client could see a mismatched version/data
     * pair if another thread modifies data between version increment and publish.
     */
    public void publishPointsToNetworkTables() {
        // Acquire write lock since we modify dataVersion
        // All operations (snapshot, version increment, publish) inside lock for atomicity
        lock.writeLock().lock();
        try {
            String[] serializedPoints = new String[points.size()];
            for (int i = 0; i < points.size(); i++) {
                serializedPoints[i] = points.get(i).toCsvRow();
            }
            int version = ++dataVersion;
            // Publish inside lock to ensure version and data are consistent
            pointsPub.set(serializedPoints);
            versionPub.set(version);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * Checks if a point already exists at the given grid position.
     *
     * @param gridRow Row index
     * @param gridCol Column index
     * @return true if a point exists at this grid position
     */
    public boolean hasPointAt(int gridRow, int gridCol) {
        lock.readLock().lock();
        try {
            return points.stream()
                    .anyMatch(p -> p.gridRow() == gridRow && p.gridCol() == gridCol);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Removes a calibration point at the given grid position.
     *
     * @param gridRow Row index
     * @param gridCol Column index
     * @return true if a point was removed, false if no point existed at that position
     */
    public boolean removePointAt(int gridRow, int gridCol) {
        boolean removed;
        lock.writeLock().lock();
        try {
            removed = points.removeIf(p -> p.gridRow() == gridRow && p.gridCol() == gridCol);
        } finally {
            lock.writeLock().unlock();
        }

        // Publish to NetworkTables after modification
        if (removed) {
            publishPointsToNetworkTables();
        }
        return removed;
    }

    /**
     * Updates an existing point at the given grid position, or adds a new one if not found.
     *
     * @param gridRow          Row index
     * @param gridCol          Column index
     * @param robotX           Robot X position in meters
     * @param robotY           Robot Y position in meters
     * @param distanceToTarget Distance to target in meters
     * @param hoodAngleDegrees Hood angle in degrees
     * @param flywheelRPM      Flywheel speed in RPM
     * @param alliance         Alliance color
     * @return true if an existing point was updated, false if a new point was added
     */
    public boolean updatePointAt(int gridRow, int gridCol, double robotX, double robotY,
            double distanceToTarget, double hoodAngleDegrees, double flywheelRPM, String alliance) {
        // Create the new point outside the lock
        String timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT);
        CalibrationPoint newPoint = new CalibrationPoint(
                timestamp, robotX, robotY, distanceToTarget,
                hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);

        boolean removed;
        lock.writeLock().lock();
        try {
            // Find and remove existing point at this grid position
            removed = points.removeIf(p -> p.gridRow() == gridRow && p.gridCol() == gridCol);

            // Add the new point
            points.add(newPoint);
        } finally {
            lock.writeLock().unlock();
        }

        // Publish to NetworkTables after modification
        publishPointsToNetworkTables();
        return removed;
    }
}
