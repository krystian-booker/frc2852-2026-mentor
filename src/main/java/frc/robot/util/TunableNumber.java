package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A double value that can be adjusted live from the dashboard.
 *
 * <p>
 * Publishes the default to SmartDashboard on construction and reads the
 * current dashboard value on every {@link #get()}. Used for calibration inputs
 * and SOTM tuning knobs so field tuning never requires a redeploy.
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        // Only publish the default if the key doesn't already have a value,
        // so a dashboard edit survives command restarts within a session.
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /** Returns the current dashboard value, or the default if unavailable. */
    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /** Overwrites the dashboard value. */
    public void set(double value) {
        SmartDashboard.putNumber(key, value);
    }
}
