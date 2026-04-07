import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Standalone code generator that reads calibration CSV data and generates
 * TurretLookupTables.java with 2D grid-based lookup tables.
 *
 * Usage: java GenerateLookupTables <csv_file> <output_java_file>
 *
 * This tool:
 * 1. Parses the calibration CSV file
 * 2. Groups points by (grid_row, grid_col) from the webapp calibration grid
 * 3. Applies IQR-based outlier rejection per grid cell
 * 4. Averages remaining measurements per cell
 * 5. Fills uncalibrated cells via nearest-neighbor interpolation
 * 6. Generates Java source file with 2D grid lookup tables
 */
public class GenerateLookupTables {

    // Grid dimensions (must match CalibrationConstants in Constants.java)
    private static final int GRID_ROWS = 17;
    private static final int GRID_COLS = 34;
    private static final double GRID_CELL_SIZE_METERS = 0.5;

    // IQR multiplier for outlier detection (1.5 is standard, use lower for stricter filtering)
    private static final double IQR_MULTIPLIER = 1.5;

    // Maximum acceptable deviation from median for small cell outlier detection (2-3 points)
    // Hood angle: max 10 degrees deviation from median
    // Flywheel RPM: max 500 RPM deviation from median
    private static final double SMALL_CELL_MAX_HOOD_DEVIATION = 10.0;
    private static final double SMALL_CELL_MAX_RPM_DEVIATION = 500.0;

    // Valid value ranges for lookup table validation (must match Constants.java)
    private static final double MIN_HOOD_ANGLE = 0.0;
    private static final double MAX_HOOD_ANGLE = 45.0;
    private static final double MIN_FLYWHEEL_RPM = 1000.0;
    private static final double MAX_FLYWHEEL_RPM = 6000.0;

    /**
     * Represents a single calibration data point.
     */
    private static class CalibrationPoint {
        final double hoodAngle;
        final double flywheelRPM;
        final String alliance;
        final int gridRow;
        final int gridCol;

        CalibrationPoint(double hoodAngle, double flywheelRPM, String alliance, int gridRow, int gridCol) {
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
            this.alliance = alliance;
            this.gridRow = gridRow;
            this.gridCol = gridCol;
        }
    }

    /**
     * Represents an averaged grid cell entry.
     */
    private static class CellEntry {
        final int row;
        final int col;
        final double hoodAngle;
        final double flywheelRPM;
        final int sampleCount;
        final int rejectedCount;

        CellEntry(int row, int col, double hoodAngle, double flywheelRPM, int sampleCount, int rejectedCount) {
            this.row = row;
            this.col = col;
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
            this.sampleCount = sampleCount;
            this.rejectedCount = rejectedCount;
        }
    }

    /**
     * Result of parsing CSV file containing points and metadata.
     */
    private static class ParseResult {
        final List<CalibrationPoint> points;
        final Set<String> alliances;
        final int parseErrorCount;
        final int validationErrorCount;
        final int totalLinesProcessed;

        ParseResult(List<CalibrationPoint> points, Set<String> alliances,
                    int parseErrorCount, int validationErrorCount, int totalLinesProcessed) {
            this.points = points;
            this.alliances = alliances;
            this.parseErrorCount = parseErrorCount;
            this.validationErrorCount = validationErrorCount;
            this.totalLinesProcessed = totalLinesProcessed;
        }

        boolean hasQualityIssues() {
            int totalErrors = parseErrorCount + validationErrorCount;
            // Quality issues if >10% of lines have errors
            return totalLinesProcessed > 0 && (double) totalErrors / totalLinesProcessed > 0.1;
        }
    }

    // Maximum error threshold before build fails (percentage of total lines)
    private static final double MAX_ERROR_THRESHOLD = 0.25; // 25%

    public static void main(String[] args) {
        if (args.length != 2) {
            System.err.println("Usage: java GenerateLookupTables <csv_file> <output_java_file>");
            System.exit(1);
        }

        String csvPath = args[0];
        String outputPath = args[1];

        try {
            ParseResult result = parseCSV(csvPath);

            // Check error threshold
            int totalErrors = result.parseErrorCount + result.validationErrorCount;
            if (result.totalLinesProcessed > 0) {
                double errorRate = (double) totalErrors / result.totalLinesProcessed;
                if (errorRate > MAX_ERROR_THRESHOLD) {
                    System.err.printf("ERROR: Too many parse/validation errors (%.1f%% of %d lines). " +
                            "Parse errors: %d, Validation errors: %d. " +
                            "Fix data quality issues before generating tables.%n",
                            errorRate * 100, result.totalLinesProcessed,
                            result.parseErrorCount, result.validationErrorCount);
                    System.exit(1);
                }
            }

            if (result.points.isEmpty()) {
                System.out.println("No calibration data found in CSV. Generating empty tables.");
                generateEmptyTables(outputPath);
                return;
            }

            // Log quality warnings prominently
            if (result.hasQualityIssues()) {
                System.out.println("========================================");
                System.out.println("WARNING: DATA QUALITY ISSUES DETECTED");
                System.out.printf("  Parse errors: %d%n", result.parseErrorCount);
                System.out.printf("  Validation errors: %d%n", result.validationErrorCount);
                System.out.printf("  Total lines processed: %d%n", result.totalLinesProcessed);
                System.out.println("========================================");
            }

            GridResult gridResult = processPoints(result.points);
            generateJavaFile(outputPath, gridResult, result);

            System.out.printf("Generated %s: %d/%d cells calibrated, %d filled by interpolation, %d outliers rejected from %d points%n",
                    outputPath, gridResult.calibratedCells, GRID_ROWS * GRID_COLS,
                    gridResult.filledCells, gridResult.totalRejected, result.points.size());

        } catch (IOException e) {
            System.err.println("Error processing calibration data: " + e.getMessage());
            System.exit(1);
        }
    }

    /**
     * Parses the calibration CSV file.
     *
     * Expected format:
     * timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,flywheel_rpm,grid_row,grid_col,alliance
     *
     * Backward compatible with older format without alliance column.
     */
    private static ParseResult parseCSV(String csvPath) throws IOException {
        List<CalibrationPoint> points = new ArrayList<>();
        Set<String> alliances = new HashSet<>();
        int parseErrorCount = 0;
        int validationErrorCount = 0;
        int totalLinesProcessed = 0;

        try (BufferedReader reader = new BufferedReader(new FileReader(csvPath))) {
            String line;
            boolean headerSkipped = false;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) {
                    continue;
                }

                // Skip header row - detect by checking for known header field names
                if (!headerSkipped) {
                    String lowerLine = line.toLowerCase();
                    // Check for known header keywords (column names from CSV format)
                    boolean isHeader = lowerLine.contains("timestamp") ||
                            lowerLine.contains("robot_x") ||
                            lowerLine.contains("robot_y") ||
                            lowerLine.contains("distance_to_target") ||
                            lowerLine.contains("hood_angle") ||
                            lowerLine.contains("flywheel_rpm") ||
                            lowerLine.contains("grid_row") ||
                            lowerLine.contains("grid_col");
                    // Set headerSkipped AFTER checking - ensures we don't mark as skipped before validation
                    headerSkipped = true;
                    if (isHeader) {
                        continue;
                    }
                    // First line was not a header; continue to process as data
                }

                String[] parts = line.split(",");
                // CSV format: timestamp,robot_x,robot_y,distance,hood,rpm,grid_row,grid_col[,alliance]
                // Require at least 8 fields (alliance optional for backward compatibility)
                if (parts.length >= 8) {
                    try {
                        double hoodAngle = Double.parseDouble(parts[4].trim());
                        double flywheelRPM = Double.parseDouble(parts[5].trim());
                        int gridRow = Integer.parseInt(parts[6].trim());
                        int gridCol = Integer.parseInt(parts[7].trim());

                        // Parse alliance (index 8) if present, otherwise default to "Unknown"
                        String alliance = "Unknown";
                        if (parts.length >= 9) {
                            alliance = parts[8].trim();
                            if (alliance.isEmpty()) {
                                alliance = "Unknown";
                            }
                        }

                        // Validate data: check for NaN/Infinity and mechanical limits
                        boolean validHoodAngle = !Double.isNaN(hoodAngle) && !Double.isInfinite(hoodAngle)
                                && hoodAngle >= MIN_HOOD_ANGLE && hoodAngle <= MAX_HOOD_ANGLE;
                        boolean validFlywheelRPM = !Double.isNaN(flywheelRPM) && !Double.isInfinite(flywheelRPM)
                                && flywheelRPM >= MIN_FLYWHEEL_RPM && flywheelRPM <= MAX_FLYWHEEL_RPM;
                        boolean validGrid = gridRow >= 0 && gridRow < GRID_ROWS
                                && gridCol >= 0 && gridCol < GRID_COLS;

                        totalLinesProcessed++;
                        if (validHoodAngle && validFlywheelRPM && validGrid) {
                            points.add(new CalibrationPoint(hoodAngle, flywheelRPM, alliance, gridRow, gridCol));
                            alliances.add(alliance);
                        } else {
                            validationErrorCount++;
                            // Provide detailed error message for failed validation
                            StringBuilder reason = new StringBuilder("Skipping line - ");
                            if (!validHoodAngle) reason.append(String.format("hood %.1f° outside %.0f-%.0f°; ",
                                    hoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE));
                            if (!validFlywheelRPM) reason.append(String.format("RPM %.0f outside %.0f-%.0f; ",
                                    flywheelRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM));
                            if (!validGrid) reason.append(String.format("grid[%d][%d] out of bounds [0-%d][0-%d]; ",
                                    gridRow, gridCol, GRID_ROWS - 1, GRID_COLS - 1));
                            System.err.println("Warning: " + reason + "line: " + line);
                        }
                    } catch (NumberFormatException e) {
                        totalLinesProcessed++;
                        parseErrorCount++;
                        System.err.println("Warning: Skipping malformed line: " + line);
                    }
                }
            }
        }

        return new ParseResult(points, alliances, parseErrorCount, validationErrorCount, totalLinesProcessed);
    }

    /**
     * Result of processing calibration points into 2D grids.
     */
    private static class GridResult {
        final double[][] hoodGrid;
        final double[][] flywheelGrid;
        final int[][] sampleCounts;
        final int[][] rejectedCounts;
        final int calibratedCells;
        final int filledCells;
        final int totalRejected;

        GridResult(double[][] hoodGrid, double[][] flywheelGrid, int[][] sampleCounts,
                   int[][] rejectedCounts, int calibratedCells, int filledCells, int totalRejected) {
            this.hoodGrid = hoodGrid;
            this.flywheelGrid = flywheelGrid;
            this.sampleCounts = sampleCounts;
            this.rejectedCounts = rejectedCounts;
            this.calibratedCells = calibratedCells;
            this.filledCells = filledCells;
            this.totalRejected = totalRejected;
        }
    }

    /**
     * Processes calibration points into 2D grid arrays.
     * Groups by (gridRow, gridCol), applies outlier rejection per cell, averages values,
     * and fills uncalibrated cells via nearest-neighbor interpolation.
     */
    private static GridResult processPoints(List<CalibrationPoint> points) {
        // Group points by (gridRow, gridCol)
        Map<Long, List<CalibrationPoint>> cells = new HashMap<>();

        for (CalibrationPoint point : points) {
            long cellKey = (long) point.gridRow * GRID_COLS + point.gridCol;
            cells.computeIfAbsent(cellKey, k -> new ArrayList<>()).add(point);
        }

        // Initialize grids with NaN
        double[][] hoodGrid = new double[GRID_ROWS][GRID_COLS];
        double[][] flywheelGrid = new double[GRID_ROWS][GRID_COLS];
        int[][] sampleCounts = new int[GRID_ROWS][GRID_COLS];
        int[][] rejectedCounts = new int[GRID_ROWS][GRID_COLS];

        for (int r = 0; r < GRID_ROWS; r++) {
            for (int c = 0; c < GRID_COLS; c++) {
                hoodGrid[r][c] = Double.NaN;
                flywheelGrid[r][c] = Double.NaN;
            }
        }

        int calibratedCells = 0;
        int totalRejected = 0;

        // Process each cell with outlier rejection
        for (Map.Entry<Long, List<CalibrationPoint>> entry : cells.entrySet()) {
            long cellKey = entry.getKey();
            int row = (int) (cellKey / GRID_COLS);
            int col = (int) (cellKey % GRID_COLS);
            List<CalibrationPoint> cellPoints = entry.getValue();
            int originalCount = cellPoints.size();

            // Apply outlier rejection based on cell size
            List<CalibrationPoint> filteredPoints = cellPoints;
            if (cellPoints.size() >= 4) {
                filteredPoints = rejectOutliers(cellPoints);
                if (filteredPoints.isEmpty()) {
                    filteredPoints = cellPoints;
                    System.out.printf("WARNING: Outlier rejection removed ALL %d points in cell [%d][%d] - using all points instead%n",
                            cellPoints.size(), row, col);
                }
            } else if (cellPoints.size() >= 2) {
                filteredPoints = rejectSmallCellOutliers(cellPoints);
                if (filteredPoints.isEmpty()) {
                    filteredPoints = cellPoints;
                    System.out.printf("WARNING: Small cell outlier rejection removed ALL %d points in cell [%d][%d] - using all points instead%n",
                            cellPoints.size(), row, col);
                }
            }

            int rejected = originalCount - filteredPoints.size();
            if (rejected > 0) {
                System.out.printf("Cell [%d][%d]: Rejected %d of %d points as outliers%n",
                        row, col, rejected, originalCount);
            }

            // Average the filtered points
            double avgHoodAngle = 0;
            double avgFlywheelRPM = 0;

            for (CalibrationPoint point : filteredPoints) {
                avgHoodAngle += point.hoodAngle;
                avgFlywheelRPM += point.flywheelRPM;
            }

            int count = filteredPoints.size();
            avgHoodAngle /= count;
            avgFlywheelRPM /= count;

            // Validate and clamp averaged values to valid ranges
            double originalHoodAngle = avgHoodAngle;
            double originalFlywheelRPM = avgFlywheelRPM;

            avgHoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, avgHoodAngle));
            avgFlywheelRPM = Math.max(MIN_FLYWHEEL_RPM, Math.min(MAX_FLYWHEEL_RPM, avgFlywheelRPM));

            if (avgHoodAngle != originalHoodAngle) {
                System.out.printf("WARNING: Cell [%d][%d] - Hood angle %.2f° clamped to %.2f° (valid range: %.0f-%.0f°)%n",
                        row, col, originalHoodAngle, avgHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
            }
            if (avgFlywheelRPM != originalFlywheelRPM) {
                System.out.printf("WARNING: Cell [%d][%d] - Flywheel RPM %.0f clamped to %.0f (valid range: %.0f-%.0f)%n",
                        row, col, originalFlywheelRPM, avgFlywheelRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM);
            }

            hoodGrid[row][col] = avgHoodAngle;
            flywheelGrid[row][col] = avgFlywheelRPM;
            sampleCounts[row][col] = count;
            rejectedCounts[row][col] = rejected;
            calibratedCells++;
            totalRejected += rejected;
        }

        // Fill uncalibrated cells using distance-weighted nearest-neighbor interpolation
        int filledCells = fillGaps(hoodGrid, flywheelGrid, sampleCounts);

        return new GridResult(hoodGrid, flywheelGrid, sampleCounts, rejectedCounts,
                calibratedCells, filledCells, totalRejected);
    }

    /**
     * Fills NaN cells in the grids using inverse-distance-weighted interpolation
     * from all calibrated neighbors. This runs at build time, so no runtime cost.
     *
     * @return Number of cells that were filled
     */
    private static int fillGaps(double[][] hoodGrid, double[][] flywheelGrid, int[][] sampleCounts) {
        // Collect all calibrated cell positions
        List<int[]> calibratedCells = new ArrayList<>();
        for (int r = 0; r < GRID_ROWS; r++) {
            for (int c = 0; c < GRID_COLS; c++) {
                if (!Double.isNaN(hoodGrid[r][c])) {
                    calibratedCells.add(new int[]{r, c});
                }
            }
        }

        if (calibratedCells.isEmpty()) {
            return 0;
        }

        int filledCount = 0;

        for (int r = 0; r < GRID_ROWS; r++) {
            for (int c = 0; c < GRID_COLS; c++) {
                if (!Double.isNaN(hoodGrid[r][c])) {
                    continue;
                }

                // Find the distance to each calibrated cell and use inverse-distance weighting
                double hoodWeightedSum = 0;
                double flywheelWeightedSum = 0;
                double weightSum = 0;

                for (int[] cal : calibratedCells) {
                    int dr = r - cal[0];
                    int dc = c - cal[1];
                    double dist = Math.sqrt(dr * dr + dc * dc);
                    // Use inverse square distance weighting for smoother interpolation
                    double weight = 1.0 / (dist * dist);
                    hoodWeightedSum += weight * hoodGrid[cal[0]][cal[1]];
                    flywheelWeightedSum += weight * flywheelGrid[cal[0]][cal[1]];
                    weightSum += weight;
                }

                hoodGrid[r][c] = hoodWeightedSum / weightSum;
                flywheelGrid[r][c] = flywheelWeightedSum / weightSum;
                filledCount++;
            }
        }

        return filledCount;
    }

    /**
     * Applies simple median-based outlier rejection for small cells (2-3 points).
     * Rejects points that deviate too far from the median in either hood angle or RPM.
     */
    private static List<CalibrationPoint> rejectSmallCellOutliers(List<CalibrationPoint> points) {
        if (points.size() < 2) {
            return new ArrayList<>(points);
        }

        // Calculate medians for hood angle and flywheel RPM
        List<Double> hoodAngles = new ArrayList<>();
        List<Double> flywheelRPMs = new ArrayList<>();
        for (CalibrationPoint point : points) {
            hoodAngles.add(point.hoodAngle);
            flywheelRPMs.add(point.flywheelRPM);
        }

        Collections.sort(hoodAngles);
        Collections.sort(flywheelRPMs);

        double medianHood = hoodAngles.size() % 2 == 0
                ? (hoodAngles.get(hoodAngles.size() / 2 - 1) + hoodAngles.get(hoodAngles.size() / 2)) / 2.0
                : hoodAngles.get(hoodAngles.size() / 2);

        double medianRPM = flywheelRPMs.size() % 2 == 0
                ? (flywheelRPMs.get(flywheelRPMs.size() / 2 - 1) + flywheelRPMs.get(flywheelRPMs.size() / 2)) / 2.0
                : flywheelRPMs.get(flywheelRPMs.size() / 2);

        // Filter points within acceptable deviation from median
        List<CalibrationPoint> filtered = new ArrayList<>();
        for (CalibrationPoint point : points) {
            double hoodDeviation = Math.abs(point.hoodAngle - medianHood);
            double rpmDeviation = Math.abs(point.flywheelRPM - medianRPM);

            if (hoodDeviation <= SMALL_CELL_MAX_HOOD_DEVIATION &&
                rpmDeviation <= SMALL_CELL_MAX_RPM_DEVIATION) {
                filtered.add(point);
            }
        }

        return filtered;
    }

    /**
     * Applies IQR-based outlier rejection to a list of calibration points.
     * Filters out points where either hood angle or flywheel RPM is outside
     * the range [Q1 - 1.5*IQR, Q3 + 1.5*IQR].
     */
    private static List<CalibrationPoint> rejectOutliers(List<CalibrationPoint> points) {
        // Extract hood angles and flywheel RPMs for IQR calculation
        List<Double> hoodAngles = new ArrayList<>();
        List<Double> flywheelRPMs = new ArrayList<>();

        for (CalibrationPoint point : points) {
            hoodAngles.add(point.hoodAngle);
            flywheelRPMs.add(point.flywheelRPM);
        }

        // Calculate IQR bounds for each metric
        double[] hoodBounds = calculateIQRBounds(hoodAngles);
        double[] rpmBounds = calculateIQRBounds(flywheelRPMs);

        // Filter points that are within bounds for both metrics
        List<CalibrationPoint> filtered = new ArrayList<>();
        for (CalibrationPoint point : points) {
            boolean hoodInBounds = point.hoodAngle >= hoodBounds[0] && point.hoodAngle <= hoodBounds[1];
            boolean rpmInBounds = point.flywheelRPM >= rpmBounds[0] && point.flywheelRPM <= rpmBounds[1];

            if (hoodInBounds && rpmInBounds) {
                filtered.add(point);
            }
        }

        return filtered;
    }

    /**
     * Calculates the IQR-based outlier bounds for a list of values.
     * Returns [lowerBound, upperBound] where values outside this range are outliers.
     */
    private static double[] calculateIQRBounds(List<Double> values) {
        if (values.isEmpty()) {
            return new double[] { Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY };
        }

        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);

        double q1 = percentile(sorted, 25);
        double q3 = percentile(sorted, 75);
        double iqr = q3 - q1;

        double lowerBound = q1 - IQR_MULTIPLIER * iqr;
        double upperBound = q3 + IQR_MULTIPLIER * iqr;

        return new double[] { lowerBound, upperBound };
    }

    /**
     * Calculates the percentile value from a sorted list.
     */
    private static double percentile(List<Double> sortedValues, double percentile) {
        if (sortedValues.isEmpty()) {
            return 0;
        }
        if (sortedValues.size() == 1) {
            return sortedValues.get(0);
        }

        double index = (percentile / 100.0) * (sortedValues.size() - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);

        if (lower == upper) {
            return sortedValues.get(lower);
        }

        double fraction = index - lower;
        return sortedValues.get(lower) * (1 - fraction) + sortedValues.get(upper) * fraction;
    }

    /**
     * Generates the Java source file with 2D grid lookup tables.
     */
    private static void generateJavaFile(String outputPath, GridResult gridResult,
            ParseResult parseResult) throws IOException {

        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
        String allianceList = String.join(", ", parseResult.alliances);
        int totalPoints = parseResult.points.size();

        try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(outputPath)))) {
            writer.println("package frc.robot.generated;");
            writer.println();
            writer.println("/**");
            writer.println(" * Auto-generated turret lookup tables from calibration data.");
            writer.println(" * This file is regenerated at build time by tools/GenerateLookupTables.java");
            writer.println(" * when calibration data exists at src/main/deploy/calibration/turret_calibration_data.csv");
            writer.println(" *");
            writer.println(" * DO NOT EDIT THIS FILE MANUALLY - changes will be overwritten.");
            writer.println(" *");
            writer.println(" * These tables are 2D grids indexed by field position (row = Y, col = X).");
            writer.println(" * Calibration data is recorded for Blue alliance. For Red alliance, the");
            writer.println(" * TurretAimingCalculator mirrors the position before lookup.");
            writer.println(" *");
            writer.println(" * Recommended interpolation method: BILINEAR interpolation between grid cells.");
            writer.println(" */");
            writer.println("public final class TurretLookupTables {");
            writer.println();
            writer.println("    private TurretLookupTables() {");
            writer.println("        // Utility class - prevent instantiation");
            writer.println("    }");
            writer.println();
            writer.println("    // ==================== QUALITY METRICS ====================");
            writer.println("    /**");
            writer.printf("     * Generated timestamp: %s%n", timestamp);
            writer.printf("     * Data points: %d (%d cells calibrated, %d filled by interpolation, %d outliers rejected)%n",
                    totalPoints, gridResult.calibratedCells, gridResult.filledCells, gridResult.totalRejected);
            writer.printf("     * Grid dimensions: %d rows x %d cols (%.1fm cell size)%n",
                    GRID_ROWS, GRID_COLS, GRID_CELL_SIZE_METERS);
            writer.printf("     * Alliances in calibration data: %s%n", allianceList);
            writer.printf("     * Parse errors: %d, Validation errors: %d%n",
                    parseResult.parseErrorCount, parseResult.validationErrorCount);
            if (parseResult.hasQualityIssues()) {
                writer.println("     * WARNING: Data quality issues detected during generation!");
            }
            writer.println("     */");
            writer.println();

            // Generate grid dimension constants
            writer.printf("    public static final int GRID_ROWS = %d;%n", GRID_ROWS);
            writer.printf("    public static final int GRID_COLS = %d;%n", GRID_COLS);
            writer.printf("    public static final double GRID_CELL_SIZE_METERS = %.1f;%n", GRID_CELL_SIZE_METERS);
            writer.println();

            // Generate hood grid
            writer.println("    /**");
            writer.println("     * Hood angle grid: [row][col] = degrees.");
            writer.println("     * Row = Y position / cell size, Col = X position / cell size.");
            writer.println("     */");
            writer.println("    public static final double[][] HOOD_GRID = {");
            writeGrid(writer, gridResult.hoodGrid, gridResult.sampleCounts, gridResult.rejectedCounts, "%.2f");
            writer.println("    };");
            writer.println();

            // Generate flywheel grid
            writer.println("    /**");
            writer.println("     * Flywheel RPM grid: [row][col] = RPM.");
            writer.println("     * Row = Y position / cell size, Col = X position / cell size.");
            writer.println("     */");
            writer.println("    public static final double[][] FLYWHEEL_GRID = {");
            writeGrid(writer, gridResult.flywheelGrid, gridResult.sampleCounts, gridResult.rejectedCounts, "%.0f");
            writer.println("    };");
            writer.println("}");
        }
    }

    /**
     * Writes a 2D grid array to the output file with sample count comments.
     */
    private static void writeGrid(PrintWriter writer, double[][] grid, int[][] sampleCounts,
                                   int[][] rejectedCounts, String valueFormat) {
        for (int r = 0; r < GRID_ROWS; r++) {
            writer.print("        { ");
            for (int c = 0; c < GRID_COLS; c++) {
                writer.printf(valueFormat, grid[r][c]);
                if (c < GRID_COLS - 1) {
                    writer.print(", ");
                }
            }
            String rowComment;
            // Count calibrated cells in this row
            int calibrated = 0;
            for (int c = 0; c < GRID_COLS; c++) {
                if (sampleCounts[r][c] > 0) {
                    calibrated++;
                }
            }
            rowComment = String.format(" // row %d (%d/%d cells calibrated)", r, calibrated, GRID_COLS);
            if (r < GRID_ROWS - 1) {
                writer.printf(" },%s%n", rowComment);
            } else {
                writer.printf(" }%s%n", rowComment);
            }
        }
    }

    /**
     * Generates an empty tables file when no calibration data exists.
     */
    private static void generateEmptyTables(String outputPath) throws IOException {
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);

        try (PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter(outputPath)))) {
            writer.println("package frc.robot.generated;");
            writer.println();
            writer.println("/**");
            writer.println(" * Auto-generated turret lookup tables from calibration data.");
            writer.println(" * This file is regenerated at build time by tools/GenerateLookupTables.java");
            writer.println(" * when calibration data exists at src/main/deploy/calibration/turret_calibration_data.csv");
            writer.println(" *");
            writer.println(" * DO NOT EDIT THIS FILE MANUALLY - changes will be overwritten.");
            writer.println(" *");
            writer.println(" * If no calibration data exists, empty tables are provided and the system");
            writer.println(" * will fall back to the default tables in TurretAimingConstants.");
            writer.println(" */");
            writer.println("public final class TurretLookupTables {");
            writer.println();
            writer.println("    private TurretLookupTables() {");
            writer.println("        // Utility class - prevent instantiation");
            writer.println("    }");
            writer.println();
            writer.println("    /**");
            writer.printf("     * Generated timestamp: %s%n", timestamp);
            writer.println("     * Data points: 0");
            writer.println("     */");
            writer.println();
            writer.printf("    public static final int GRID_ROWS = %d;%n", GRID_ROWS);
            writer.printf("    public static final int GRID_COLS = %d;%n", GRID_COLS);
            writer.printf("    public static final double GRID_CELL_SIZE_METERS = %.1f;%n", GRID_CELL_SIZE_METERS);
            writer.println();
            writer.println("    /**");
            writer.println("     * Hood angle grid: empty indicates no calibration data - use fallback tables.");
            writer.println("     */");
            writer.println("    public static final double[][] HOOD_GRID = {};");
            writer.println();
            writer.println("    /**");
            writer.println("     * Flywheel RPM grid: empty indicates no calibration data - use fallback tables.");
            writer.println("     */");
            writer.println("    public static final double[][] FLYWHEEL_GRID = {};");
            writer.println("}");
        }
    }
}
