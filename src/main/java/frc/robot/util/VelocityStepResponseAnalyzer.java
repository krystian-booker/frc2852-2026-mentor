package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Records velocity-vs-time samples during step tests and computes step response metrics: rise time,
 * overshoot, settling time, steady-state error, and disturbance recovery time.
 */
public class VelocityStepResponseAnalyzer {

  private final double targetVelocity;
  private final double startVelocity;
  private final double stepSize;
  private final double tolerance;
  private final List<Double> timestamps = new ArrayList<>();
  private final List<Double> velocities = new ArrayList<>();
  private final double startTime;

  /**
   * @param startVelocity velocity before step command (RPS)
   * @param targetVelocity commanded velocity after step (RPS)
   * @param tolerance settling tolerance in RPS
   */
  public VelocityStepResponseAnalyzer(
      double startVelocity, double targetVelocity, double tolerance) {
    this.startVelocity = startVelocity;
    this.targetVelocity = targetVelocity;
    this.stepSize = targetVelocity - startVelocity;
    this.tolerance = tolerance;
    this.startTime = System.nanoTime() / 1e9;
  }

  /** Record a sample. Call every robot cycle. */
  public void addSample(double velocityRPS) {
    double now = System.nanoTime() / 1e9;
    timestamps.add(now - startTime);
    velocities.add(velocityRPS);
  }

  /** Time to reach 90% of step size (seconds). Returns -1 if never reached. */
  public double getRiseTime() {
    double threshold = startVelocity + stepSize * 0.9;
    boolean positive = stepSize > 0;
    for (int i = 0; i < velocities.size(); i++) {
      if (positive ? velocities.get(i) >= threshold : velocities.get(i) <= threshold) {
        return timestamps.get(i);
      }
    }
    return -1;
  }

  /** Maximum overshoot beyond target as a percentage of step size. */
  public double getOvershootPercent() {
    if (velocities.isEmpty() || Math.abs(stepSize) < 1e-6) {
      return 0;
    }
    double maxOvershoot = 0;
    boolean positive = stepSize > 0;
    for (double vel : velocities) {
      double overshoot = positive ? (vel - targetVelocity) : (targetVelocity - vel);
      if (overshoot > maxOvershoot) {
        maxOvershoot = overshoot;
      }
    }
    return (maxOvershoot / Math.abs(stepSize)) * 100.0;
  }

  /**
   * Time until velocity stays within tolerance of target permanently (seconds). Returns -1 if not
   * settled by end of recording.
   */
  public double getSettlingTime() {
    // Walk backwards to find the last sample outside tolerance
    for (int i = velocities.size() - 1; i >= 0; i--) {
      if (Math.abs(velocities.get(i) - targetVelocity) > tolerance) {
        if (i + 1 < timestamps.size()) {
          return timestamps.get(i + 1);
        }
        return -1; // never settled
      }
    }
    // All samples were within tolerance
    return timestamps.isEmpty() ? -1 : timestamps.get(0);
  }

  /** Error at the end of the recording window (RPS). */
  public double getSteadyStateError() {
    if (velocities.isEmpty()) {
      return Double.NaN;
    }
    return Math.abs(velocities.get(velocities.size() - 1) - targetVelocity);
  }

  /**
   * Time from a disturbance until velocity re-enters the tolerance band (seconds). Searches from
   * the given timestamp forward for the first sample back within tolerance.
   *
   * @param disturbanceTimestamp time of disturbance relative to analyzer start (seconds)
   * @return recovery time in seconds, or -1 if never recovered
   */
  public double getRecoveryTime(double disturbanceTimestamp) {
    for (int i = 0; i < timestamps.size(); i++) {
      if (timestamps.get(i) >= disturbanceTimestamp) {
        // Find first sample back within tolerance after disturbance
        for (int j = i; j < timestamps.size(); j++) {
          if (Math.abs(velocities.get(j) - targetVelocity) <= tolerance) {
            return timestamps.get(j) - disturbanceTimestamp;
          }
        }
        return -1; // never recovered
      }
    }
    return -1;
  }

  public int getSampleCount() {
    return velocities.size();
  }

  public double getTargetVelocity() {
    return targetVelocity;
  }

  public double getStartVelocity() {
    return startVelocity;
  }
}
