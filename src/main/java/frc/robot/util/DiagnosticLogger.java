package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DiagnosticConstants;

/**
 * Simple CSV logger for diagnostic data. Writes timestamped CSV files to the roboRIO filesystem. Togglable at runtime
 * via SmartDashboard.
 */
public class DiagnosticLogger {
    private final String name;
    private final String[] columnHeaders;
    private final String dashboardKey;

    private PrintWriter writer;
    private boolean errorReported = false;
    private String currentFilePath = "";

    public DiagnosticLogger(String name, String[] columnHeaders) {
        this.name = name;
        this.columnHeaders = columnHeaders;
        this.dashboardKey = "Diagnostics/" + name;
        SmartDashboard.putBoolean(dashboardKey, DiagnosticConstants.TURRET_LOGGING_DEFAULT_ENABLED);
        SmartDashboard.putString(dashboardKey + "/File", "");
    }

    /** Create a new timestamped CSV file and write the header row. */
    public void open() {
        try {
            File dir = new File(DiagnosticConstants.LOG_DIRECTORY);
            if (!dir.exists()) {
                dir.mkdirs();
            }

            String timestamp = LocalDateTime.now()
                    .format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
            String filename = name + "_" + timestamp + ".csv";
            File file = new File(dir, filename);
            currentFilePath = file.getAbsolutePath();

            writer = new PrintWriter(new BufferedWriter(new FileWriter(file)));
            writer.println(String.join(",", columnHeaders));
            errorReported = false;
            SmartDashboard.putString(dashboardKey + "/File", currentFilePath);
        } catch (Exception e) {
            reportError("Failed to open log file: " + e.getMessage());
            writer = null;
        }
    }

    /** Append one row of numeric values. Checks the SmartDashboard toggle. */
    public void logRow(double... values) {
        if (writer == null) {
            return;
        }
        // if (!SmartDashboard.getBoolean(dashboardKey, false)) {
        // return;
        // }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(values[i]);
        }
        writer.println(sb.toString());
    }

    /** Flush and close the current file. */
    public void close() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (Exception e) {
                reportError("Failed to close log file: " + e.getMessage());
            }
            writer = null;
        }
    }

    public boolean isOpen() {
        return writer != null;
    }

    public String getCurrentFilePath() {
        return currentFilePath;
    }

    private void reportError(String message) {
        if (!errorReported) {
            DriverStation.reportWarning("[DiagnosticLogger] " + message, false);
            errorReported = true;
        }
    }
}
