package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import frc.robot.Constants.CalibrationConstants;

/**
 * Manages calibration data recording and CSV file I/O on the roboRIO.
 * Stores calibration points in memory and writes to CSV file on demand.
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
        this.errorPub = NetworkTableInstance.getDefault()
                .getTable("TurretCalibration")
                .getStringTopic("Error")
                .publish();
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

        String timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT);
        CalibrationPoint point = new CalibrationPoint(
                timestamp, robotX, robotY, distanceToTarget,
                hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);
        points.add(point);
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
        points.add(point);
    }

    /**
     * Writes all buffered points to the CSV file.
     *
     * @return true if the file was written successfully, false otherwise
     */
    public boolean saveToFile() {
        try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(filePath, false)))) {
            writer.println(CSV_HEADER);
            for (CalibrationPoint point : points) {
                writer.println(point.toCsvRow());
            }
            writer.flush();
            errorPub.set(""); // Clear any previous error
            return true;
        } catch (IOException e) {
            String errorMsg = "Failed to save calibration data: " + e.getMessage();
            System.err.println(errorMsg);
            errorPub.set(errorMsg);
            return false;
        }
    }

    /**
     * Clears all buffered points and deletes the CSV file if it exists.
     */
    public void clearData() {
        points.clear();
        File file = new File(filePath);
        if (file.exists()) {
            if (!file.delete() && file.exists()) {
                String errorMsg = "Failed to delete calibration file: " + filePath;
                System.err.println(errorMsg);
                errorPub.set(errorMsg);
            }
        }
    }

    /**
     * Gets the number of points currently in the buffer.
     *
     * @return Number of saved points
     */
    public int getPointCount() {
        return points.size();
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
        return new ArrayList<>(points);
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
     * Checks if a point already exists at the given grid position.
     *
     * @param gridRow Row index
     * @param gridCol Column index
     * @return true if a point exists at this grid position
     */
    public boolean hasPointAt(int gridRow, int gridCol) {
        return points.stream()
                .anyMatch(p -> p.gridRow() == gridRow && p.gridCol() == gridCol);
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
        // Find and remove existing point at this grid position
        boolean removed = points.removeIf(p -> p.gridRow() == gridRow && p.gridCol() == gridCol);

        // Add the new point
        String timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT);
        CalibrationPoint newPoint = new CalibrationPoint(
                timestamp, robotX, robotY, distanceToTarget,
                hoodAngleDegrees, flywheelRPM, gridRow, gridCol, alliance);
        points.add(newPoint);

        return removed;
    }
}
