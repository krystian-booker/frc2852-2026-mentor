package frc.robot.energy;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Class for logging current, power, and energy usage by subsystem. */
public class BatteryLogger {
  private double totalCurrent = 0.0;
  private double driveCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;
  private double batteryVoltage = 12.6;
  private double rioCurrent = 0.0;

  private Map<String, Double> subsystemCurrents = new HashMap<>();
  private Map<String, Double> subsystemPowers = new HashMap<>();
  private Map<String, Double> subsystemEnergies = new HashMap<>();

  public void setBatteryVoltage(double voltage) {
    this.batteryVoltage = voltage;
  }

  public void setRioCurrent(double current) {
    this.rioCurrent = current;
  }

  public double getDriveCurrent() {
    return driveCurrent;
  }

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  public void reportCurrentUsage(String key, boolean drive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.abs(amp);
    if (drive) {
      driveCurrent += totalAmps;
    }

    double power = totalAmps * batteryVoltage;
    double energy = power * Constants.loopPeriodSecs;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);

    // Roll up to parent keys (e.g., "Drive/FL" rolls up to "Drive")
    String[] keys = key.split("/|-");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsystemCurrents.merge(subkey, totalAmps, Double::sum);
      subsystemPowers.merge(subkey, power, Double::sum);
      subsystemEnergies.merge(subkey, energy, Double::sum);
    }
  }

  public void periodicAfterScheduler() {
    // Add control system baseline currents
    reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
    reportCurrentUsage("Controls/CANcoders", false, 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", false, 0.04);
    reportCurrentUsage("Controls/CANivore", false, 0.03);
    reportCurrentUsage("Controls/Radio", false, 0.5);

    // Log totals
    Logger.recordOutput("EnergyLogger/Current", totalCurrent);
    Logger.recordOutput("EnergyLogger/Power", totalPower);
    Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy));

    // Log per-subsystem and reset per-loop values
    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue());
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue());
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()));
    }

    resetTotals();
  }

  public void resetTotals() {
    totalPower = 0.0;
    totalCurrent = 0.0;
    driveCurrent = 0.0;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}
