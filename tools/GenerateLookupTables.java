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
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Standalone code generator that reads calibration CSV data and generates
 * TurretLookupTables.java with optimized interpolation tables.
 *
 * Usage: java GenerateLookupTables <csv_file> <output_java_file>
 *
 * This tool:
 * 1. Parses the calibration CSV file
 * 2. Groups points by distance (rounded to 0.25m buckets)
 * 3. Applies IQR-based outlier rejection to filter bad measurements
 * 4. Averages remaining measurements at the same distance
 * 5. Sorts by distance for efficient interpolation
 * 6. Generates Java source file with lookup tables
 */
public class GenerateLookupTables {

    /**
     * Distance bucket size for grouping measurements (meters).
     *
     * This is intentionally smaller than the UI grid cell size (0.5m in CalibrationConstants).
     * The UI grid provides visual guidance for positioning the robot, while this finer
     * bucket size preserves distance precision when processing calibration data.
     *
     * Multiple samples taken within the same UI grid cell may end up in different
     * distance buckets, providing better interpolation accuracy in the final lookup tables.
     */
    private static final double DISTANCE_BUCKET_SIZE = 0.25;

    // IQR multiplier for outlier detection (1.5 is standard, use lower for stricter filtering)
    private static final double IQR_MULTIPLIER = 1.5;

    // Maximum acceptable deviation from median for small bucket outlier detection (2-3 points)
    // Hood angle: max 10 degrees deviation from median
    // Flywheel RPM: max 500 RPM deviation from median
    private static final double SMALL_BUCKET_MAX_HOOD_DEVIATION = 10.0;
    private static final double SMALL_BUCKET_MAX_RPM_DEVIATION = 500.0;

    // Valid value ranges for lookup table validation (must match Constants.java)
    private static final double MIN_HOOD_ANGLE = 0.0;
    private static final double MAX_HOOD_ANGLE = 45.0;
    private static final double MIN_FLYWHEEL_RPM = 1000.0;
    private static final double MAX_FLYWHEEL_RPM = 6000.0;

    /**
     * Represents a single calibration data point.
     */
    private static class CalibrationPoint {
        final double distance;
        final double hoodAngle;
        final double flywheelRPM;
        final String alliance;

        CalibrationPoint(double distance, double hoodAngle, double flywheelRPM, String alliance) {
            this.distance = distance;
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
            this.alliance = alliance;
        }
    }

    /**
     * Represents an averaged lookup table entry.
     */
    private static class TableEntry {
        final double distance;
        final double hoodAngle;
        final double flywheelRPM;
        final int sampleCount;
        final int rejectedCount;

        TableEntry(double distance, double hoodAngle, double flywheelRPM, int sampleCount, int rejectedCount) {
            this.distance = distance;
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

            List<TableEntry> entries = processPoints(result.points);
            generateJavaFile(outputPath, entries, result);

            int totalRejected = entries.stream().mapToInt(e -> e.rejectedCount).sum();
            System.out.printf("Generated %s with %d lookup entries from %d calibration points (%d outliers rejected)%n",
                    outputPath, entries.size(), result.points.size(), totalRejected);

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
                        double distance = Double.parseDouble(parts[3].trim());
                        double hoodAngle = Double.parseDouble(parts[4].trim());
                        double flywheelRPM = Double.parseDouble(parts[5].trim());

                        // Parse alliance (index 8) if present, otherwise default to "Unknown"
                        String alliance = "Unknown";
                        if (parts.length >= 9) {
                            alliance = parts[8].trim();
                            if (alliance.isEmpty()) {
                                alliance = "Unknown";
                            }
                        }

                        // Validate data: check for NaN/Infinity and mechanical limits
                        // These limits must match Constants.java (HoodConstants and FlywheelConstants)
                        boolean validDistance = !Double.isNaN(distance) && !Double.isInfinite(distance) && distance > 0;
                        boolean validHoodAngle = !Double.isNaN(hoodAngle) && !Double.isInfinite(hoodAngle)
                                && hoodAngle >= MIN_HOOD_ANGLE && hoodAngle <= MAX_HOOD_ANGLE;
                        boolean validFlywheelRPM = !Double.isNaN(flywheelRPM) && !Double.isInfinite(flywheelRPM)
                                && flywheelRPM >= MIN_FLYWHEEL_RPM && flywheelRPM <= MAX_FLYWHEEL_RPM;

                        totalLinesProcessed++;
                        if (validDistance && validHoodAngle && validFlywheelRPM) {
                            points.add(new CalibrationPoint(distance, hoodAngle, flywheelRPM, alliance));
                            alliances.add(alliance);
                        } else {
                            validationErrorCount++;
                            // Provide detailed error message for failed validation
                            StringBuilder reason = new StringBuilder("Skipping line - ");
                            if (!validDistance) reason.append("distance invalid; ");
                            if (!validHoodAngle) reason.append(String.format("hood %.1f° outside %.0f-%.0f°; ",
                                    hoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE));
                            if (!validFlywheelRPM) reason.append(String.format("RPM %.0f outside %.0f-%.0f; ",
                                    flywheelRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM));
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
     * Processes calibration points into averaged lookup table entries.
     * Groups by distance bucket, applies IQR-based outlier rejection, and averages values.
     */
    private static List<TableEntry> processPoints(List<CalibrationPoint> points) {
        // Group points by distance bucket
        Map<Integer, List<CalibrationPoint>> buckets = new HashMap<>();

        for (CalibrationPoint point : points) {
            int bucketKey = (int) Math.floor(point.distance / DISTANCE_BUCKET_SIZE);
            buckets.computeIfAbsent(bucketKey, k -> new ArrayList<>()).add(point);
        }

        // Process each bucket with outlier rejection
        List<TableEntry> entries = new ArrayList<>();

        for (Map.Entry<Integer, List<CalibrationPoint>> entry : buckets.entrySet()) {
            List<CalibrationPoint> bucketPoints = entry.getValue();
            int originalCount = bucketPoints.size();

            // Apply outlier rejection based on bucket size
            List<CalibrationPoint> filteredPoints = bucketPoints;
            double bucketDistance = (entry.getKey() + 0.5) * DISTANCE_BUCKET_SIZE;
            if (bucketPoints.size() >= 4) {
                // Use IQR-based outlier rejection for larger buckets
                filteredPoints = rejectOutliers(bucketPoints);
                // Fall back to all points if filtering removes everything
                if (filteredPoints.isEmpty()) {
                    filteredPoints = bucketPoints;
                    System.out.printf("WARNING: Outlier rejection removed ALL %d points in bucket at %.2fm - using all points instead%n",
                            bucketPoints.size(), bucketDistance);
                }
            } else if (bucketPoints.size() >= 2) {
                // Use simple median-deviation outlier detection for small buckets (2-3 points)
                filteredPoints = rejectSmallBucketOutliers(bucketPoints);
                if (filteredPoints.isEmpty()) {
                    filteredPoints = bucketPoints;
                    System.out.printf("WARNING: Small bucket outlier rejection removed ALL %d points in bucket at %.2fm - using all points instead%n",
                            bucketPoints.size(), bucketDistance);
                }
            }

            int rejectedCount = originalCount - filteredPoints.size();
            if (rejectedCount > 0) {
                System.out.printf("Bucket %.2fm: Rejected %d of %d points as outliers%n",
                        bucketDistance, rejectedCount, originalCount);
            }

            // Average the filtered points
            double avgDistance = 0;
            double avgHoodAngle = 0;
            double avgFlywheelRPM = 0;

            for (CalibrationPoint point : filteredPoints) {
                avgDistance += point.distance;
                avgHoodAngle += point.hoodAngle;
                avgFlywheelRPM += point.flywheelRPM;
            }

            int count = filteredPoints.size();
            avgDistance /= count;
            avgHoodAngle /= count;
            avgFlywheelRPM /= count;

            // Validate and clamp averaged values to valid ranges
            boolean hoodClamped = false;
            boolean rpmClamped = false;
            double originalHoodAngle = avgHoodAngle;
            double originalFlywheelRPM = avgFlywheelRPM;

            if (avgHoodAngle < MIN_HOOD_ANGLE) {
                avgHoodAngle = MIN_HOOD_ANGLE;
                hoodClamped = true;
            } else if (avgHoodAngle > MAX_HOOD_ANGLE) {
                avgHoodAngle = MAX_HOOD_ANGLE;
                hoodClamped = true;
            }

            if (avgFlywheelRPM < MIN_FLYWHEEL_RPM) {
                avgFlywheelRPM = MIN_FLYWHEEL_RPM;
                rpmClamped = true;
            } else if (avgFlywheelRPM > MAX_FLYWHEEL_RPM) {
                avgFlywheelRPM = MAX_FLYWHEEL_RPM;
                rpmClamped = true;
            }

            if (hoodClamped) {
                System.out.printf("WARNING: Bucket %.2fm - Hood angle %.2f° clamped to %.2f° (valid range: %.0f-%.0f°)%n",
                        bucketDistance, originalHoodAngle, avgHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
            }
            if (rpmClamped) {
                System.out.printf("WARNING: Bucket %.2fm - Flywheel RPM %.0f clamped to %.0f (valid range: %.0f-%.0f)%n",
                        bucketDistance, originalFlywheelRPM, avgFlywheelRPM, MIN_FLYWHEEL_RPM, MAX_FLYWHEEL_RPM);
            }

            entries.add(new TableEntry(avgDistance, avgHoodAngle, avgFlywheelRPM, count, rejectedCount));
        }

        // Sort by distance for efficient interpolation
        entries.sort(Comparator.comparingDouble(e -> e.distance));

        // Check for coverage gaps that could cause poor interpolation
        for (int i = 1; i < entries.size(); i++) {
            double gap = entries.get(i).distance - entries.get(i - 1).distance;
            if (gap > 1.0) {
                System.out.printf("WARNING: Coverage gap of %.2fm between %.2fm and %.2fm%n",
                        gap, entries.get(i - 1).distance, entries.get(i).distance);
            }
        }

        return entries;
    }

    /**
     * Applies simple median-based outlier rejection for small buckets (2-3 points).
     * Rejects points that deviate too far from the median in either hood angle or RPM.
     * This is more robust than IQR for very small sample sizes.
     */
    private static List<CalibrationPoint> rejectSmallBucketOutliers(List<CalibrationPoint> points) {
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

            if (hoodDeviation <= SMALL_BUCKET_MAX_HOOD_DEVIATION &&
                rpmDeviation <= SMALL_BUCKET_MAX_RPM_DEVIATION) {
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

        int n = sorted.size();
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
     * Generates the Java source file with lookup tables.
     */
    private static void generateJavaFile(String outputPath, List<TableEntry> entries,
            ParseResult parseResult) throws IOException {

        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
        String allianceList = String.join(", ", parseResult.alliances);
        int totalRejected = entries.stream().mapToInt(e -> e.rejectedCount).sum();
        int totalPoints = parseResult.points.size();

        // Calculate coverage gaps for quality metrics
        List<String> coverageGaps = new ArrayList<>();
        for (int i = 1; i < entries.size(); i++) {
            double gap = entries.get(i).distance - entries.get(i - 1).distance;
            if (gap > 1.0) {
                coverageGaps.add(String.format("%.2fm-%.2fm", entries.get(i - 1).distance, entries.get(i).distance));
            }
        }

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
            writer.println(" * Note: These tables are distance-based and work for both alliances.");
            writer.println(" * The TurretAimingCalculator handles alliance-specific target positions,");
            writer.println(" * so the same hood/flywheel settings apply regardless of alliance.");
            writer.println(" *");
            writer.println(" * Recommended interpolation method: LINEAR interpolation between distance points.");
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
            writer.printf("     * Data points: %d (averaged into %d entries, %d outliers rejected)%n",
                    totalPoints, entries.size(), totalRejected);
            writer.printf("     * Distance bucket size: %.2fm%n", DISTANCE_BUCKET_SIZE);
            writer.printf("     * Alliances in calibration data: %s%n", allianceList);
            writer.printf("     * Parse errors: %d, Validation errors: %d%n",
                    parseResult.parseErrorCount, parseResult.validationErrorCount);
            if (parseResult.hasQualityIssues()) {
                writer.println("     * WARNING: Data quality issues detected during generation!");
            }
            if (!coverageGaps.isEmpty()) {
                writer.printf("     * Coverage gaps (>1m): %s%n", String.join(", ", coverageGaps));
            } else {
                writer.println("     * Coverage gaps: None detected");
            }
            writer.println("     */");
            writer.println();

            // Generate hood lookup table
            writer.println("    /**");
            writer.println("     * Hood angle lookup table.");
            writer.println("     * Format: { {distance_meters, hood_angle_degrees}, ... }");
            writer.println("     * Sorted by distance for efficient interpolation.");
            writer.println("     */");
            writer.println("    public static final double[][] HOOD_LOOKUP_TABLE = {");

            for (int i = 0; i < entries.size(); i++) {
                TableEntry entry = entries.get(i);
                String comma = (i < entries.size() - 1) ? "," : "";
                String comment = entry.rejectedCount > 0
                        ? String.format(" // %d samples (%d rejected)", entry.sampleCount, entry.rejectedCount)
                        : String.format(" // %d samples", entry.sampleCount);
                writer.printf("        { %.3f, %.2f }%s%s%n",
                        entry.distance, entry.hoodAngle, comma, comment);
            }

            writer.println("    };");
            writer.println();

            // Generate flywheel lookup table
            writer.println("    /**");
            writer.println("     * Flywheel RPM lookup table.");
            writer.println("     * Format: { {distance_meters, flywheel_rpm}, ... }");
            writer.println("     * Sorted by distance for efficient interpolation.");
            writer.println("     */");
            writer.println("    public static final double[][] FLYWHEEL_LOOKUP_TABLE = {");

            for (int i = 0; i < entries.size(); i++) {
                TableEntry entry = entries.get(i);
                String comma = (i < entries.size() - 1) ? "," : "";
                String comment = entry.rejectedCount > 0
                        ? String.format(" // %d samples (%d rejected)", entry.sampleCount, entry.rejectedCount)
                        : String.format(" // %d samples", entry.sampleCount);
                writer.printf("        { %.3f, %.0f }%s%s%n",
                        entry.distance, entry.flywheelRPM, comma, comment);
            }

            writer.println("    };");
            writer.println("}");
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
            writer.println("    /**");
            writer.println("     * Hood angle lookup table.");
            writer.println("     * Format: { {distance_meters, hood_angle_degrees}, ... }");
            writer.println("     * Sorted by distance for efficient interpolation.");
            writer.println("     *");
            writer.println("     * Empty array indicates no calibration data - use fallback tables.");
            writer.println("     */");
            writer.println("    public static final double[][] HOOD_LOOKUP_TABLE = {};");
            writer.println();
            writer.println("    /**");
            writer.println("     * Flywheel RPM lookup table.");
            writer.println("     * Format: { {distance_meters, flywheel_rpm}, ... }");
            writer.println("     * Sorted by distance for efficient interpolation.");
            writer.println("     *");
            writer.println("     * Empty array indicates no calibration data - use fallback tables.");
            writer.println("     */");
            writer.println("    public static final double[][] FLYWHEEL_LOOKUP_TABLE = {};");
            writer.println("}");
        }
    }
}
