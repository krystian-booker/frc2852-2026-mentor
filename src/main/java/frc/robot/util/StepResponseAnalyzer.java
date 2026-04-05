package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Records position-vs-time samples during step tests and computes step response metrics: rise time,
 * overshoot, settling time, and steady-state error.
 */
public class StepResponseAnalyzer {

  private final double targetPosition;
  private final double startPosition;
  private final double stepSize;
  private final double tolerance;
  private final List<Double> timestamps = new ArrayList<>();
  private final List<Double> positions = new ArrayList<>();
  private final double startTime;

  /**
   * @param startPosition position before step command
   * @param targetPosition commanded position after step
   * @param tolerance settling tolerance in the same units as position
   */
  public StepResponseAnalyzer(double startPosition, double targetPosition, double tolerance) {
    this.startPosition = startPosition;
    this.targetPosition = targetPosition;
    this.stepSize = targetPosition - startPosition;
    this.tolerance = tolerance;
    this.startTime = System.nanoTime() / 1e9;
  }

  /** Record a sample. Call every robot cycle. */
  public void addSample(double position) {
    double now = System.nanoTime() / 1e9;
    timestamps.add(now - startTime);
    positions.add(position);
  }

  /** Time to reach 90% of step size (seconds). Returns -1 if never reached. */
  public double getRiseTime() {
    double threshold = startPosition + stepSize * 0.9;
    boolean positive = stepSize > 0;
    for (int i = 0; i < positions.size(); i++) {
      if (positive ? positions.get(i) >= threshold : positions.get(i) <= threshold) {
        return timestamps.get(i);
      }
    }
    return -1;
  }

  /** Maximum overshoot beyond target as a percentage of step size. */
  public double getOvershootPercent() {
    if (positions.isEmpty() || Math.abs(stepSize) < 1e-6) {
      return 0;
    }
    double maxOvershoot = 0;
    boolean positive = stepSize > 0;
    for (double pos : positions) {
      double overshoot = positive ? (pos - targetPosition) : (targetPosition - pos);
      if (overshoot > maxOvershoot) {
        maxOvershoot = overshoot;
      }
    }
    return (maxOvershoot / Math.abs(stepSize)) * 100.0;
  }

  /**
   * Time until position stays within tolerance of target permanently (seconds). Returns -1 if not
   * settled by end of recording.
   */
  public double getSettlingTime() {
    // Walk backwards to find the last sample outside tolerance
    for (int i = positions.size() - 1; i >= 0; i--) {
      if (Math.abs(positions.get(i) - targetPosition) > tolerance) {
        if (i + 1 < timestamps.size()) {
          return timestamps.get(i + 1);
        }
        return -1; // never settled
      }
    }
    // All samples were within tolerance
    return timestamps.isEmpty() ? -1 : timestamps.get(0);
  }

  /** Error at the end of the recording window (position units). */
  public double getSteadyStateError() {
    if (positions.isEmpty()) {
      return Double.NaN;
    }
    return Math.abs(positions.get(positions.size() - 1) - targetPosition);
  }

  public int getSampleCount() {
    return positions.size();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getStartPosition() {
    return startPosition;
  }
}
