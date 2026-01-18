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
        ensureDirectoryExists();
    }

    /**
     * Ensures the parent directory for the calibration file exists.
     */
    private void ensureDirectoryExists() {
        File file = new File(filePath);
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            parentDir.mkdirs();
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
     */
    public void addPoint(double robotX, double robotY, double distanceToTarget,
            double hoodAngleDegrees, double flywheelRPM, int gridRow, int gridCol, String alliance) {
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
            return true;
        } catch (IOException e) {
            System.err.println("Failed to save calibration data: " + e.getMessage());
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
            file.delete();
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
}
