import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
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
 * 3. Averages multiple measurements at the same distance
 * 4. Sorts by distance for efficient interpolation
 * 5. Generates Java source file with lookup tables
 */
public class GenerateLookupTables {

    // Distance bucket size for grouping measurements (meters)
    private static final double DISTANCE_BUCKET_SIZE = 0.25;

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

        TableEntry(double distance, double hoodAngle, double flywheelRPM, int sampleCount) {
            this.distance = distance;
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
            this.sampleCount = sampleCount;
        }
    }

    /**
     * Result of parsing CSV file containing points and metadata.
     */
    private static class ParseResult {
        final List<CalibrationPoint> points;
        final Set<String> alliances;

        ParseResult(List<CalibrationPoint> points, Set<String> alliances) {
            this.points = points;
            this.alliances = alliances;
        }
    }

    public static void main(String[] args) {
        if (args.length != 2) {
            System.err.println("Usage: java GenerateLookupTables <csv_file> <output_java_file>");
            System.exit(1);
        }

        String csvPath = args[0];
        String outputPath = args[1];

        try {
            ParseResult result = parseCSV(csvPath);

            if (result.points.isEmpty()) {
                System.out.println("No calibration data found in CSV. Generating empty tables.");
                generateEmptyTables(outputPath);
                return;
            }

            List<TableEntry> entries = processPoints(result.points);
            generateJavaFile(outputPath, entries, result.points.size(), result.alliances);

            System.out.printf("Generated %s with %d lookup entries from %d calibration points%n",
                    outputPath, entries.size(), result.points.size());

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

        try (BufferedReader reader = new BufferedReader(new FileReader(csvPath))) {
            String line;
            boolean headerSkipped = false;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) {
                    continue;
                }

                // Skip header row
                if (!headerSkipped) {
                    headerSkipped = true;
                    if (line.startsWith("timestamp")) {
                        continue;
                    }
                }

                String[] parts = line.split(",");
                if (parts.length >= 6) {
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

                        // Validate data
                        if (distance > 0 && hoodAngle >= 0 && flywheelRPM >= 0) {
                            points.add(new CalibrationPoint(distance, hoodAngle, flywheelRPM, alliance));
                            alliances.add(alliance);
                        }
                    } catch (NumberFormatException e) {
                        System.err.println("Warning: Skipping malformed line: " + line);
                    }
                }
            }
        }

        return new ParseResult(points, alliances);
    }

    /**
     * Processes calibration points into averaged lookup table entries.
     * Groups by distance bucket and averages values.
     */
    private static List<TableEntry> processPoints(List<CalibrationPoint> points) {
        // Group points by distance bucket
        Map<Integer, List<CalibrationPoint>> buckets = new HashMap<>();

        for (CalibrationPoint point : points) {
            int bucketKey = (int) Math.round(point.distance / DISTANCE_BUCKET_SIZE);
            buckets.computeIfAbsent(bucketKey, k -> new ArrayList<>()).add(point);
        }

        // Average each bucket
        List<TableEntry> entries = new ArrayList<>();

        for (Map.Entry<Integer, List<CalibrationPoint>> entry : buckets.entrySet()) {
            List<CalibrationPoint> bucketPoints = entry.getValue();

            double avgDistance = 0;
            double avgHoodAngle = 0;
            double avgFlywheelRPM = 0;

            for (CalibrationPoint point : bucketPoints) {
                avgDistance += point.distance;
                avgHoodAngle += point.hoodAngle;
                avgFlywheelRPM += point.flywheelRPM;
            }

            int count = bucketPoints.size();
            avgDistance /= count;
            avgHoodAngle /= count;
            avgFlywheelRPM /= count;

            entries.add(new TableEntry(avgDistance, avgHoodAngle, avgFlywheelRPM, count));
        }

        // Sort by distance for efficient interpolation
        entries.sort(Comparator.comparingDouble(e -> e.distance));

        return entries;
    }

    /**
     * Generates the Java source file with lookup tables.
     */
    private static void generateJavaFile(String outputPath, List<TableEntry> entries, int totalPoints,
            Set<String> alliances) throws IOException {

        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
        String allianceList = String.join(", ", alliances);

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
            writer.println(" */");
            writer.println("public final class TurretLookupTables {");
            writer.println();
            writer.println("    private TurretLookupTables() {");
            writer.println("        // Utility class - prevent instantiation");
            writer.println("    }");
            writer.println();
            writer.println("    /**");
            writer.printf("     * Generated timestamp: %s%n", timestamp);
            writer.printf("     * Data points: %d (averaged into %d entries)%n", totalPoints, entries.size());
            writer.printf("     * Distance bucket size: %.2fm%n", DISTANCE_BUCKET_SIZE);
            writer.printf("     * Alliances in calibration data: %s%n", allianceList);
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
                writer.printf("        { %.3f, %.2f }%s // %d samples%n",
                        entry.distance, entry.hoodAngle, comma, entry.sampleCount);
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
                writer.printf("        { %.3f, %.0f }%s // %d samples%n",
                        entry.distance, entry.flywheelRPM, comma, entry.sampleCount);
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
