package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@SuppressWarnings("unused")
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.tuningMode ? dashboardNumber.get() : defaultValue;
    }
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
