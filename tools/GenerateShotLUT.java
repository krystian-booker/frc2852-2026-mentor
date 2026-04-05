import java.io.FileWriter;
import java.io.PrintWriter;

/**
 * Build-time tool that pre-generates the shot lookup table by running the same RK4 projectile
 * physics that ProjectileSimulator uses. The output is a Java source file that can be compiled into
 * the robot project, eliminating the multi-minute startup delay on the roboRIO.
 *
 * <p>Usage: java GenerateShotLUT <output-file-path>
 *
 * <p>If you change physics parameters (ball mass, drag, hood angles, etc.), update them here AND in
 * AimingCalculator/Constants so both stay in sync.
 */
public class GenerateShotLUT {

  // ── Robot parameters (must match AimingCalculator + Constants) ──────────
  static final double BALL_MASS_KG = 0.235;
  static final double BALL_DIAMETER_M = 0.1778; // 7 inches
  static final double DRAG_COEFF = 0.47;
  static final double MAGNUS_COEFF = 0.2;
  static final double AIR_DENSITY = 1.225;
  static final double EXIT_HEIGHT_M = 0.5;
  static final double WHEEL_DIAMETER_M = 0.1016; // 4 inches
  static final double TARGET_HEIGHT_M = 2.64;
  static final double SLIP_FACTOR = 0.6;
  static final double DT = 0.001;
  static final double RPM_MIN = 1500;
  static final double RPM_MAX = 6000;
  static final int BINARY_SEARCH_ITERS = 25;
  static final double MAX_SIM_TIME = 5.0;
  static final double MAGNUS_SIGN = 1.0; // +1 topspin

  // Hood angle mapping (must match HoodConstants)
  static final double ACTUAL_ANGLE_AT_ZERO = 70.0;
  static final double MECHANISM_MIN = 0.0;
  static final double MECHANISM_MAX = 25.0;
  static final double ANGLE_STEP = 1.0;

  // Distance range
  static final double DIST_MIN = 0.50;
  static final double DIST_MAX = 17.0;
  static final double DIST_STEP = 0.05;

  // ── Precomputed aero constants ─────────────────────────────────────────
  static final double AREA;
  static final double K_DRAG;
  static final double K_MAGNUS;

  static {
    AREA = Math.PI * (BALL_DIAMETER_M / 2.0) * (BALL_DIAMETER_M / 2.0);
    K_DRAG = (AIR_DENSITY * DRAG_COEFF * AREA) / (2.0 * BALL_MASS_KG);
    K_MAGNUS = (AIR_DENSITY * MAGNUS_COEFF * AREA) / (2.0 * BALL_MASS_KG);
  }

  // ── RK4 physics (mirrors ProjectileSimulator exactly) ──────────────────

  static double exitVelocity(double rpm) {
    return SLIP_FACTOR * rpm * Math.PI * WHEEL_DIAMETER_M / 60.0;
  }

  static double[] derivatives(double[] state) {
    double svx = state[2];
    double svz = state[3];
    double speed = Math.hypot(svx, svz);
    double ax = -K_DRAG * speed * svx;
    double az = -9.81 - K_DRAG * speed * svz + MAGNUS_SIGN * K_MAGNUS * speed * speed;
    return new double[] {svx, svz, ax, az};
  }

  static double[] addScaled(double[] base, double[] delta, double scale) {
    return new double[] {
      base[0] + delta[0] * scale,
      base[1] + delta[1] * scale,
      base[2] + delta[2] * scale,
      base[3] + delta[3] * scale
    };
  }

  /** Returns {zAtTarget, tof, reachedTarget (1.0 or 0.0)} */
  static double[] simulate(double rpm, double targetDistanceM, double launchAngleDeg) {
    double v0 = exitVelocity(rpm);
    double launchRad = Math.toRadians(launchAngleDeg);
    double vx = v0 * Math.cos(launchRad);
    double vz = v0 * Math.sin(launchRad);

    double x = 0, z = EXIT_HEIGHT_M;
    double t = 0;

    while (t < MAX_SIM_TIME) {
      double prevX = x, prevZ = z;

      double[] state = {x, z, vx, vz};
      double[] k1 = derivatives(state);
      double[] s2 = addScaled(state, k1, DT / 2.0);
      double[] k2 = derivatives(s2);
      double[] s3 = addScaled(state, k2, DT / 2.0);
      double[] k3 = derivatives(s3);
      double[] s4 = addScaled(state, k3, DT);
      double[] k4 = derivatives(s4);

      x += DT / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      z += DT / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      vx += DT / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vz += DT / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      t += DT;

      if (x >= targetDistanceM) {
        double frac = (targetDistanceM - prevX) / (x - prevX);
        double zAtTarget = prevZ + frac * (z - prevZ);
        double tofAtTarget = t - DT + frac * DT;
        return new double[] {zAtTarget, tofAtTarget, 1.0};
      }

      if (z < 0) {
        return new double[] {0, t, 0.0};
      }
    }
    return new double[] {0, MAX_SIM_TIME, 0.0};
  }

  /**
   * Binary search for RPM at a given angle that lands at target height. Returns {rpm, tof,
   * reachable}.
   */
  static double[] findRPMForDistance(double distanceM, double launchAngleDeg) {
    double heightTolerance = 0.02;
    double lo = RPM_MIN, hi = RPM_MAX;

    double[] maxCheck = simulate(hi, distanceM, launchAngleDeg);
    if (maxCheck[2] < 0.5) {
      return new double[] {0, 0, 0}; // unreachable
    }

    double bestRpm = hi;
    double bestTof = maxCheck[1];
    double bestError = Math.abs(maxCheck[0] - TARGET_HEIGHT_M);

    for (int i = 0; i < BINARY_SEARCH_ITERS; i++) {
      double mid = (lo + hi) / 2.0;
      double[] result = simulate(mid, distanceM, launchAngleDeg);

      if (result[2] < 0.5) {
        lo = mid;
        continue;
      }

      double error = result[0] - TARGET_HEIGHT_M;
      double absError = Math.abs(error);

      if (absError < bestError) {
        bestRpm = mid;
        bestTof = result[1];
        bestError = absError;
      }

      if (absError < heightTolerance) {
        return new double[] {mid, result[1], 1.0};
      }

      if (error > 0) {
        hi = mid;
      } else {
        lo = mid;
      }
    }

    return new double[] {bestRpm, bestTof, bestError < 0.10 ? 1.0 : 0.0};
  }

  // ── Main: generate LUT and write Java source ───────────────────────────

  public static void main(String[] args) throws Exception {
    if (args.length < 1) {
      System.err.println("Usage: java GenerateShotLUT <output-file-path>");
      System.exit(1);
    }

    String outputPath = args[0];
    long startMs = System.currentTimeMillis();

    // Collect LUT entries: {distance, rpm, mechanismAngle, tof}
    java.util.List<double[]> entries = new java.util.ArrayList<>();

    for (double distance = DIST_MIN; distance <= DIST_MAX + 0.001; distance += DIST_STEP) {
      distance = Math.round(distance * 100.0) / 100.0;

      double bestRPM = Double.MAX_VALUE;
      double bestMechAngle = 0;
      double bestTOF = 0;
      boolean found = false;

      for (double mech = MECHANISM_MIN; mech <= MECHANISM_MAX + 0.001; mech += ANGLE_STEP) {
        double physicsAngle = ACTUAL_ANGLE_AT_ZERO - mech;
        double[] result = findRPMForDistance(distance, physicsAngle);
        if (result[2] > 0.5 && result[0] < bestRPM) {
          bestRPM = result[0];
          bestMechAngle = mech;
          bestTOF = result[1];
          found = true;
        }
      }

      if (found) {
        entries.add(new double[] {distance, bestRPM, bestMechAngle, bestTOF});
      }
    }

    long elapsed = System.currentTimeMillis() - startMs;

    // Write Java source file
    java.io.File outFile = new java.io.File(outputPath);
    outFile.getParentFile().mkdirs();

    try (PrintWriter pw = new PrintWriter(new FileWriter(outFile))) {
      pw.println("package frc.robot.generated;");
      pw.println();
      pw.println("import frc.robot.util.firecontrol.ShotLUT;");
      pw.println("import frc.robot.util.firecontrol.ShotParameters;");
      pw.println();
      pw.println("/**");
      pw.println(" * Pre-generated shot lookup table. Built at compile time by");
      pw.println(" * tools/GenerateShotLUT.java using RK4 projectile physics.");
      pw.println(" *");
      pw.printf(
          " * <p>Generated: %d entries, %.1fs generation time.%n",
          entries.size(), elapsed / 1000.0);
      pw.println(" *");
      pw.println(" * <p>DO NOT EDIT — regenerate by running {@code ./gradlew generateShotLUT}");
      pw.println(" * or by changing tools/GenerateShotLUT.java (triggers automatic rebuild).");
      pw.println(" */");
      pw.println("public final class GeneratedShotLUT {");
      pw.println();
      pw.println("    private GeneratedShotLUT() {}");
      pw.println();
      pw.println("    /** Number of entries in the LUT. */");
      pw.printf("    public static final int ENTRY_COUNT = %d;%n", entries.size());
      pw.println();
      pw.println("    /** Create the pre-computed shot lookup table. */");
      pw.println("    public static ShotLUT create() {");
      pw.println("        ShotLUT lut = new ShotLUT();");

      for (double[] e : entries) {
        pw.printf(
            "        lut.put(%.2f, new ShotParameters(%.1f, %.1f, %.4f));%n",
            e[0], e[1], e[2], e[3]);
      }

      pw.println("        return lut;");
      pw.println("    }");
      pw.println("}");
    }

    // Summary to stdout (visible in Gradle output)
    System.out.printf(
        "[GenerateShotLUT] %d entries, %.2fm to %.2fm, generated in %.1fs%n",
        entries.size(),
        entries.isEmpty() ? 0 : entries.get(0)[0],
        entries.isEmpty() ? 0 : entries.get(entries.size() - 1)[0],
        elapsed / 1000.0);
    System.out.printf("[GenerateShotLUT] Written to %s%n", outputPath);
  }
}
