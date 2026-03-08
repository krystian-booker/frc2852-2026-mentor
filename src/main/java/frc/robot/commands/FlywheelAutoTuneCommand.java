package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.util.VelocityStepResponseAnalyzer;

/**
 * Automated PID tuning command for the flywheel. Runs as a 4-phase state machine:
 * <ol>
 * <li>Feedforward characterization (kS, kV, kA) using current (Amps)</li>
 * <li>PID tuning via step-response binary search</li>
 * <li>Disturbance rejection testing and optimization</li>
 * <li>Validation and report</li>
 * </ol>
 * Total runtime ~5.5 minutes. On interruption, restores original gains.
 */
public class FlywheelAutoTuneCommand extends Command {

    // Safety limits
    private static final double MAX_CHARACTERIZATION_CURRENT = 40.0; // Amps
    private static final double MAX_VELOCITY_RPS = 100.0; // 6000 RPM
    private static final double STATOR_CURRENT_ABORT = 100.0; // Amps
    private static final int MAX_STATOR_VIOLATIONS = 20; // consecutive cycles before abort
    private static final double MIN_RPM_FOR_OPPOSING_CURRENT = 500.0; // RPM

    // Phase 1 constants
    private static final double KS_RAMP_RATE = 0.1; // Amps per cycle
    private static final double KS_VELOCITY_THRESHOLD = 0.05; // RPS
    private static final int KS_CONSECUTIVE_SAMPLES = 3;
    private static final double KS_ABORT_CURRENT = 20.0; // Amps
    private static final double KV_SETTLE_SECONDS = 3.0;
    private static final double KA_MEASURE_SECONDS = 0.5;

    // Phase 2 constants
    private static final double SETTLE_SECONDS = 3.0;
    private static final double STEP_RECORD_SECONDS = 5.0;
    private static final double TEST_RPM = 3000.0;
    private static final double VELOCITY_TOLERANCE_RPS = 50.0 / 60.0; // ~0.83 RPS

    // Phase 3 constants
    private static final double DISTURBANCE_CURRENT = -15.0; // Amps (opposing)
    private static final double DISTURBANCE_DURATION_MS = 40.0;
    private static final double RECOVERY_TARGET_MS = 250.0;
    private static final int MAX_DISTURBANCE_RETUNE_ITERATIONS = 3;
    private static final double DISTURBANCE_MULTI_INTERVAL_S = 0.5;

    private static final String PREFIX = "FlywheelAutoTune/";

    private final Flywheel flywheel;

    // State machine
    private enum Phase {
        FEEDFORWARD, PID_TUNING, DISTURBANCE_REJECTION, VALIDATION, DONE
    }

    private Phase currentPhase;
    private int subStep;
    private int cycleCount;

    // Feedforward results
    private double tunedKs;
    private double tunedKv;
    private double tunedKa;

    // PID results
    private double tunedKp;
    private double tunedKi;
    private double tunedKd;

    // Phase 1 (FF) working state
    private double ksRampCurrent;
    private int ksConsecutiveCount;
    private double[] kvTestCurrents;
    private double[] kvMeasuredVelocities;
    private int kvTestIndex;
    private double kvAccumVelocity;
    private int kvAccumCount;
    private double kaPrevVelocity;
    private double kaAccelSum;
    private int kaAccelCount;

    // Phase 2 (PID) working state
    private double kpLow;
    private double kpHigh;
    private double kpTest;
    private int binarySearchIteration;
    private boolean kpSearchDone;
    private double[] kdCandidates;
    private int kdTestIndex;
    private double bestKdOvershoot;
    private double bestKd;
    private double[] kiCandidates;
    private int kiTestIndex;
    private double bestKiError;
    private double bestKiOvershoot;
    private double bestKi;

    // Phase 3 (Disturbance) working state
    private int disturbanceRetuneIteration;
    private double disturbancePulseStart;
    private boolean disturbancePulseActive;
    private double lastRecoveryTimeMs;
    private double[] multiVelocityRPMs;
    private int multiVelocityIndex;
    private int multiPulseCount;
    private double multiPulseLastTime;

    // Phase 4 (Validation) working state
    private double[][] validationTests; // [startRPM, endRPM]
    private int validationIndex;
    private boolean validationDisturbanceDone;

    // Safety
    private int consecutiveStatorViolations;

    // Step response recording
    private VelocityStepResponseAnalyzer analyzer;
    private double stepStartTime;
    private boolean settling;

    public FlywheelAutoTuneCommand(Flywheel flywheel) {
        this.flywheel = flywheel;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        System.out.println("=== FLYWHEEL AUTO-TUNE STARTING ===");
        publishStatus("Starting...");

        currentPhase = Phase.FEEDFORWARD;
        subStep = 0;
        cycleCount = 0;

        // Initialize feedforward state
        ksRampCurrent = 0;
        ksConsecutiveCount = 0;

        tunedKs = 0;
        tunedKv = 0;
        tunedKa = 0.01;
        tunedKp = 0;
        tunedKi = 0;
        tunedKd = 0;

        consecutiveStatorViolations = 0;

        // Start from neutral
        flywheel.stop();
    }

    @Override
    public void execute() {
        cycleCount++;

        // Safety checks
        double velocityRPS = flywheel.getVelocityRPS();
        double statorCurrent = flywheel.getStatorCurrent();
        SmartDashboard.putNumber(PREFIX + "Velocity RPS", velocityRPS);
        SmartDashboard.putNumber(PREFIX + "Stator Current", statorCurrent);

        if (Math.abs(velocityRPS) > MAX_VELOCITY_RPS) {
            System.err.println("FLYWHEEL AUTO-TUNE: Velocity exceeded " + (MAX_VELOCITY_RPS * 60) + " RPM. Aborting.");
            cancel();
            return;
        }

        if (Math.abs(statorCurrent) > STATOR_CURRENT_ABORT) {
            consecutiveStatorViolations++;
            if (consecutiveStatorViolations >= MAX_STATOR_VIOLATIONS) {
                System.err.println("FLYWHEEL AUTO-TUNE: Stator current exceeded " + STATOR_CURRENT_ABORT
                        + "A for " + MAX_STATOR_VIOLATIONS + " cycles. Aborting.");
                cancel();
                return;
            }
        } else {
            consecutiveStatorViolations = 0;
        }

        switch (currentPhase) {
        case FEEDFORWARD:
            executeFeedforward();
            break;
        case PID_TUNING:
            executePidTuning();
            break;
        case DISTURBANCE_REJECTION:
            executeDisturbanceRejection();
            break;
        case VALIDATION:
            executeValidation();
            break;
        case DONE:
            break;
        }
    }

    // ===== PHASE 1: FEEDFORWARD CHARACTERIZATION =====

    private void executeFeedforward() {
        SmartDashboard.putString(PREFIX + "Phase", "Feedforward");

        switch (subStep) {
        case 0: // Settle at neutral
            flywheel.stop();
            if (cycleCount > 100) { // 2 seconds at 50Hz
                subStep = 1;
                ksRampCurrent = 0;
                ksConsecutiveCount = 0;
                cycleCount = 0;
                publishStatus("Phase 1: Finding kS");
            }
            break;

        case 1: // kS ramp (forward only - flywheel is unidirectional)
            ksRampCurrent += KS_RAMP_RATE;
            if (ksRampCurrent > KS_ABORT_CURRENT) {
                System.err.println("FLYWHEEL AUTO-TUNE: kS ramp exceeded " + KS_ABORT_CURRENT + "A without motion. Aborting.");
                cancel();
                return;
            }
            flywheel.applyCurrent(ksRampCurrent);
            if (Math.abs(flywheel.getVelocityRPS()) > KS_VELOCITY_THRESHOLD) {
                ksConsecutiveCount++;
            } else {
                ksConsecutiveCount = 0;
            }
            if (ksConsecutiveCount >= KS_CONSECUTIVE_SAMPLES) {
                tunedKs = ksRampCurrent;
                flywheel.stop();
                SmartDashboard.putNumber(PREFIX + "kS", tunedKs);
                publishStatus("Phase 1: kS = " + fmt(tunedKs) + " - preparing kV tests");

                // Prepare kV test currents (ascending, capped at MAX_CHARACTERIZATION_CURRENT)
                kvTestCurrents = new double[] {
                    Math.min(tunedKs + 5, MAX_CHARACTERIZATION_CURRENT),
                    Math.min(tunedKs + 10, MAX_CHARACTERIZATION_CURRENT),
                    Math.min(tunedKs + 15, MAX_CHARACTERIZATION_CURRENT),
                    Math.min(tunedKs + 20, MAX_CHARACTERIZATION_CURRENT)
                };
                kvMeasuredVelocities = new double[4];
                kvTestIndex = 0;
                subStep = 2;
                cycleCount = 0;
            }
            break;

        case 2: // kV test: apply current, wait for steady state, measure
            flywheel.applyCurrent(kvTestCurrents[kvTestIndex]);
            double kvElapsed = cycleCount / 50.0;

            if (kvElapsed < KV_SETTLE_SECONDS) {
                // Settling phase
                publishStatus("Phase 1: kV test " + (kvTestIndex + 1) + "/4 at "
                        + fmt(kvTestCurrents[kvTestIndex]) + "A - settling");
            } else if (kvElapsed < KV_SETTLE_SECONDS + 0.5) {
                // Measure phase (0.5s)
                kvAccumVelocity += Math.abs(flywheel.getVelocityRPS());
                kvAccumCount++;
            } else {
                // Done with this current level
                double avgVel = kvAccumCount > 0 ? kvAccumVelocity / kvAccumCount : 0;
                kvMeasuredVelocities[kvTestIndex] = avgVel;
                kvTestIndex++;

                if (kvTestIndex < 4) {
                    // Next ascending level (no coast-down needed)
                    kvAccumVelocity = 0;
                    kvAccumCount = 0;
                    cycleCount = 0;
                } else {
                    // Compute kV from all measurements
                    double kvSum = 0;
                    int kvValid = 0;
                    for (int i = 0; i < 4; i++) {
                        if (kvMeasuredVelocities[i] > 0.01) {
                            kvSum += (kvTestCurrents[i] - tunedKs) / kvMeasuredVelocities[i];
                            kvValid++;
                        }
                    }
                    tunedKv = kvValid > 0 ? kvSum / kvValid : 0.12;
                    flywheel.stop();
                    SmartDashboard.putNumber(PREFIX + "kV", tunedKv);
                    publishStatus("Phase 1: kV = " + fmt(tunedKv) + " - measuring kA");
                    subStep = 3;
                    cycleCount = 0;
                }
            }
            break;

        case 3: // Settle before kA test
            flywheel.stop();
            if (cycleCount > 100) { // 2s settle
                subStep = 4;
                cycleCount = 0;
                kaPrevVelocity = 0;
                kaAccelSum = 0;
                kaAccelCount = 0;
            }
            break;

        case 4: // kA: step current from rest, measure acceleration
            double kaCurrent = Math.min(tunedKs + 10, MAX_CHARACTERIZATION_CURRENT);
            flywheel.applyCurrent(kaCurrent);
            double kaElapsed = cycleCount / 50.0;

            if (kaElapsed > 0.02 && kaElapsed < KA_MEASURE_SECONDS) {
                double vel = Math.abs(flywheel.getVelocityRPS());
                if (kaPrevVelocity > 0) {
                    double accel = (vel - kaPrevVelocity) * 50.0; // dv/dt at 50Hz
                    if (accel > 0) {
                        kaAccelSum += accel;
                        kaAccelCount++;
                    }
                }
                kaPrevVelocity = vel;
            } else if (kaElapsed >= KA_MEASURE_SECONDS) {
                flywheel.stop();
                if (kaAccelCount > 0) {
                    double avgAccel = kaAccelSum / kaAccelCount;
                    if (avgAccel > 0.1) {
                        tunedKa = (kaCurrent - tunedKs) / avgAccel;
                    }
                }
                // Clamp kA to reasonable range
                tunedKa = Math.max(0.001, Math.min(1.0, tunedKa));
                SmartDashboard.putNumber(PREFIX + "kA", tunedKa);
                publishStatus("Phase 1 COMPLETE: kS=" + fmt(tunedKs) + " kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));
                System.out.println("Phase 1 Results: kS=" + fmt(tunedKs) + " kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));

                subStep = 0;
                cycleCount = 0;
                currentPhase = Phase.PID_TUNING;
                initPidTuning();
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== PHASE 2: PID TUNING =====

    private void initPidTuning() {
        kpTest = 0.5;
        kpLow = 0;
        kpHigh = 0;
        binarySearchIteration = 0;
        kpSearchDone = false;
        tunedKp = 0.5;
        tunedKd = 0;
        tunedKi = 0;

        kdCandidates = null;
        kdTestIndex = 0;
        bestKdOvershoot = Double.MAX_VALUE;
        bestKd = 0;

        kiCandidates = null;
        kiTestIndex = 0;
        bestKiError = Double.MAX_VALUE;
        bestKiOvershoot = Double.MAX_VALUE;
        bestKi = 0;

        // Apply feedforward gains with starting PID
        flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, kpTest, 0, 0);
        publishStatus("Phase 2: PID tuning - starting kP search at " + kpTest);
    }

    private void executePidTuning() {
        SmartDashboard.putString(PREFIX + "Phase", "PID Tuning");

        switch (subStep) {
        case 0: // Settle at neutral before kP step test
            flywheel.stop();
            if (cycleCount > (int)(SETTLE_SECONDS * 50)) {
                subStep = 1;
                cycleCount = 0;
                startVelocityStepTest(0, TEST_RPM / 60.0); // 0 → 3000 RPM
            }
            break;

        case 1: // kP doubling / binary search: run step test
            if (runVelocityStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                SmartDashboard.putNumber(PREFIX + "StepResponse/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "StepResponse/RiseTime", analyzer.getRiseTime());
                SmartDashboard.putNumber(PREFIX + "StepResponse/SettlingTime", analyzer.getSettlingTime());
                publishStatus("Phase 2: kP=" + fmt(kpTest) + " overshoot=" + String.format("%.1f", overshoot) + "%");

                if (!kpSearchDone) {
                    if (overshoot > 5.0) {
                        // Too much overshoot - start binary search
                        kpHigh = kpTest;
                        kpLow = kpTest / 2.0; // previous value didn't overshoot
                        kpSearchDone = true;
                        binarySearchIteration = 0;
                    } else {
                        // Not enough overshoot, keep doubling
                        if (kpTest >= 32.0) {
                            tunedKp = kpTest;
                            finishKpSearch();
                            break;
                        }
                        kpTest *= 2;
                    }
                }

                if (kpSearchDone) {
                    if (overshoot > 5.0) {
                        kpHigh = kpTest;
                    } else {
                        kpLow = kpTest;
                    }

                    if (binarySearchIteration >= 6) {
                        tunedKp = kpLow;
                        finishKpSearch();
                        break;
                    }
                    kpTest = (kpLow + kpHigh) / 2.0;
                    binarySearchIteration++;
                }

                // Apply new kP and run another step test
                flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, kpTest, 0, 0);
                flywheel.stop();
                subStep = 2;
                cycleCount = 0;
            }
            break;

        case 2: // Settle before next kP test
            flywheel.stop();
            if (cycleCount > (int)(SETTLE_SECONDS * 50)) {
                subStep = 1;
                cycleCount = 0;
                startVelocityStepTest(0, TEST_RPM / 60.0);
            }
            break;

        case 3: // kD search: run step test
            if (runVelocityStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                SmartDashboard.putNumber(PREFIX + "StepResponse/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "StepResponse/SettlingTime", analyzer.getSettlingTime());

                if (overshoot < 10.0 && overshoot < bestKdOvershoot) {
                    bestKdOvershoot = overshoot;
                    bestKd = kdCandidates[kdTestIndex];
                }
                publishStatus("Phase 2: kD=" + fmt(kdCandidates[kdTestIndex])
                        + " overshoot=" + String.format("%.1f", overshoot) + "%");

                kdTestIndex++;
                if (kdTestIndex >= kdCandidates.length) {
                    tunedKd = bestKd;
                    SmartDashboard.putNumber(PREFIX + "kP", tunedKp);
                    SmartDashboard.putNumber(PREFIX + "kD", tunedKd);
                    publishStatus("Phase 2: kP=" + fmt(tunedKp) + " kD=" + fmt(tunedKd) + " - starting kI search");
                    initKiSearch();
                } else {
                    // Test next kD
                    flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, 0, kdCandidates[kdTestIndex]);
                    flywheel.stop();
                    subStep = 4;
                    cycleCount = 0;
                }
            }
            break;

        case 4: // Settle before next kD test
            flywheel.stop();
            if (cycleCount > (int)(SETTLE_SECONDS * 50)) {
                subStep = 3;
                cycleCount = 0;
                startVelocityStepTest(0, TEST_RPM / 60.0);
            }
            break;

        case 5: // kI search: run step test
            if (runVelocityStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double ssError = analyzer.getSteadyStateError();
                SmartDashboard.putNumber(PREFIX + "StepResponse/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "StepResponse/SSError", ssError);

                if (overshoot < 5.0 && ssError < bestKiError) {
                    bestKiError = ssError;
                    bestKiOvershoot = overshoot;
                    bestKi = kiCandidates[kiTestIndex];
                }
                publishStatus("Phase 2: kI=" + fmt(kiCandidates[kiTestIndex])
                        + " ssError=" + fmt(ssError) + " overshoot=" + String.format("%.1f", overshoot) + "%");

                kiTestIndex++;
                if (kiTestIndex >= kiCandidates.length) {
                    tunedKi = bestKi;
                    SmartDashboard.putNumber(PREFIX + "kI", tunedKi);
                    publishStatus("Phase 2 COMPLETE: kP=" + fmt(tunedKp) + " kI=" + fmt(tunedKi) + " kD=" + fmt(tunedKd));
                    System.out.println("Phase 2 Results: kP=" + fmt(tunedKp) + " kI=" + fmt(tunedKi) + " kD=" + fmt(tunedKd));

                    // Apply final PID and move to disturbance rejection
                    flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, tunedKi, tunedKd);
                    flywheel.stop();
                    currentPhase = Phase.DISTURBANCE_REJECTION;
                    subStep = 0;
                    cycleCount = 0;
                    initDisturbanceRejection();
                } else {
                    // Test next kI
                    flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, kiCandidates[kiTestIndex], tunedKd);
                    flywheel.stop();
                    subStep = 6;
                    cycleCount = 0;
                }
            }
            break;

        case 6: // Settle before next kI test
            flywheel.stop();
            if (cycleCount > (int)(SETTLE_SECONDS * 50)) {
                subStep = 5;
                cycleCount = 0;
                startVelocityStepTest(0, TEST_RPM / 60.0);
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    private void finishKpSearch() {
        SmartDashboard.putNumber(PREFIX + "kP", tunedKp);
        publishStatus("Phase 2: kP=" + fmt(tunedKp) + " - starting kD search");

        // Setup kD candidates
        kdCandidates = new double[] { 0, tunedKp * 0.01, tunedKp * 0.02, tunedKp * 0.05, tunedKp * 0.1 };
        kdTestIndex = 0;
        bestKdOvershoot = Double.MAX_VALUE;
        bestKd = 0;

        // Apply first kD candidate
        flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, 0, kdCandidates[0]);
        flywheel.stop();
        subStep = 4;
        cycleCount = 0;
    }

    private void initKiSearch() {
        kiCandidates = new double[] { 0, tunedKp * 0.001, tunedKp * 0.005, tunedKp * 0.01 };
        kiTestIndex = 0;
        bestKiError = Double.MAX_VALUE;
        bestKiOvershoot = Double.MAX_VALUE;
        bestKi = 0;

        flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, kiCandidates[0], tunedKd);
        flywheel.stop();
        subStep = 6;
        cycleCount = 0;
    }

    // ===== PHASE 3: DISTURBANCE REJECTION =====

    private void initDisturbanceRejection() {
        disturbanceRetuneIteration = 0;
        disturbancePulseActive = false;
        lastRecoveryTimeMs = 0;
        multiVelocityRPMs = new double[] { 2000, 3000, 4000, 5000 };
        multiVelocityIndex = 0;
        multiPulseCount = 0;
        multiPulseLastTime = 0;
        publishStatus("Phase 3: Disturbance rejection testing");
    }

    private void executeDisturbanceRejection() {
        SmartDashboard.putString(PREFIX + "Phase", "Disturbance Rejection");

        switch (subStep) {
        case 0: // Spin up to test RPM
            flywheel.setVelocity(TEST_RPM);
            if (cycleCount > (int)(SETTLE_SECONDS * 50) && flywheel.atSetpoint()) {
                subStep = 1;
                cycleCount = 0;
                disturbancePulseActive = false;
                // Start recording for disturbance
                analyzer = new VelocityStepResponseAnalyzer(
                        TEST_RPM / 60.0, TEST_RPM / 60.0, VELOCITY_TOLERANCE_RPS);
                publishStatus("Phase 3: Single disturbance test");
            }
            break;

        case 1: // Single disturbance test
            double now = System.nanoTime() / 1e9;
            analyzer.addSample(flywheel.getVelocityRPS());

            if (!disturbancePulseActive && cycleCount > 25) { // 0.5s after recording starts
                // Apply disturbance pulse only if velocity is high enough
                if (flywheel.getCurrentVelocityRPM() > MIN_RPM_FOR_OPPOSING_CURRENT) {
                    disturbancePulseActive = true;
                    disturbancePulseStart = now;
                    flywheel.applyCurrent(DISTURBANCE_CURRENT);
                }
            }

            if (disturbancePulseActive) {
                double pulseElapsedMs = (now - disturbancePulseStart) * 1000.0;
                if (pulseElapsedMs >= DISTURBANCE_DURATION_MS) {
                    // Switch back to velocity control
                    flywheel.setVelocity(TEST_RPM);
                    disturbancePulseActive = false;
                }
            }

            // Record for 2s after pulse
            if (cycleCount > 25 + (int)(2.0 * 50)) {
                // Evaluate recovery
                double disturbanceTime = 25.0 / 50.0; // approximate disturbance timestamp
                lastRecoveryTimeMs = analyzer.getRecoveryTime(disturbanceTime) * 1000.0;
                SmartDashboard.putNumber(PREFIX + "RecoveryTime ms", lastRecoveryTimeMs);
                publishStatus("Phase 3: Recovery time = " + String.format("%.0f", lastRecoveryTimeMs) + "ms");
                subStep = 2;
                cycleCount = 0;
            }
            break;

        case 2: // Evaluate & adjust
            if (lastRecoveryTimeMs > RECOVERY_TARGET_MS && lastRecoveryTimeMs > 0
                    && disturbanceRetuneIteration < MAX_DISTURBANCE_RETUNE_ITERATIONS) {
                // Bump kP by 25%
                tunedKp *= 1.25;
                // Add small kI if not already present
                if (tunedKi == 0) {
                    tunedKi = tunedKp * 0.005;
                }
                flywheel.applyTuningConfig(tunedKs, tunedKv, tunedKa, tunedKp, tunedKi, tunedKd);
                disturbanceRetuneIteration++;
                publishStatus("Phase 3: Bumped kP to " + fmt(tunedKp) + " - retesting (iteration "
                        + disturbanceRetuneIteration + ")");
                // Re-test
                subStep = 0;
                cycleCount = 0;
            } else {
                // Move to multi-velocity test
                publishStatus("Phase 3: Starting multi-velocity disturbance tests");
                multiVelocityIndex = 0;
                subStep = 3;
                cycleCount = 0;
            }
            break;

        case 3: // Multi-velocity spin up
            if (multiVelocityIndex >= multiVelocityRPMs.length) {
                // All done
                publishStatus("Phase 3 COMPLETE: kP=" + fmt(tunedKp) + " kI=" + fmt(tunedKi) + " kD=" + fmt(tunedKd));
                System.out.println("Phase 3 Results: kP=" + fmt(tunedKp) + " kI=" + fmt(tunedKi) + " kD=" + fmt(tunedKd));

                flywheel.stop();
                currentPhase = Phase.VALIDATION;
                subStep = 0;
                cycleCount = 0;
                initValidation();
                break;
            }
            flywheel.setVelocity(multiVelocityRPMs[multiVelocityIndex]);
            if (cycleCount > (int)(SETTLE_SECONDS * 50) && flywheel.atSetpoint()) {
                subStep = 4;
                cycleCount = 0;
                multiPulseCount = 0;
                multiPulseLastTime = 0;
                double targetRPS = multiVelocityRPMs[multiVelocityIndex] / 60.0;
                analyzer = new VelocityStepResponseAnalyzer(targetRPS, targetRPS, VELOCITY_TOLERANCE_RPS);
                publishStatus("Phase 3: Multi-pulse at " + (int) multiVelocityRPMs[multiVelocityIndex] + " RPM");
            }
            break;

        case 4: // Multi-velocity pulse test (3 consecutive pulses, 0.5s apart)
            double nowMulti = System.nanoTime() / 1e9;
            double elapsedS = cycleCount / 50.0;
            analyzer.addSample(flywheel.getVelocityRPS());

            if (multiPulseCount < 3) {
                double nextPulseTime = multiPulseCount * DISTURBANCE_MULTI_INTERVAL_S + 0.5; // 0.5s initial settle
                if (elapsedS >= nextPulseTime && !disturbancePulseActive) {
                    if (flywheel.getCurrentVelocityRPM() > MIN_RPM_FOR_OPPOSING_CURRENT) {
                        disturbancePulseActive = true;
                        disturbancePulseStart = nowMulti;
                        multiPulseLastTime = elapsedS;
                        flywheel.applyCurrent(DISTURBANCE_CURRENT);
                    }
                }
                if (disturbancePulseActive) {
                    double pulseElapsedMs = (nowMulti - disturbancePulseStart) * 1000.0;
                    if (pulseElapsedMs >= DISTURBANCE_DURATION_MS) {
                        flywheel.setVelocity(multiVelocityRPMs[multiVelocityIndex]);
                        disturbancePulseActive = false;
                        multiPulseCount++;
                    }
                }
            }

            // Wait for recovery after last pulse + 0.5s
            if (multiPulseCount >= 3 && elapsedS > multiPulseLastTime + 0.5) {
                // Check recovery for last pulse
                double recoveryMs = analyzer.getRecoveryTime(multiPulseLastTime) * 1000.0;
                SmartDashboard.putNumber(PREFIX + "MultiRecovery ms", recoveryMs);

                if (recoveryMs > RECOVERY_TARGET_MS && recoveryMs > 0) {
                    publishStatus("Phase 3: WARN - Recovery " + String.format("%.0f", recoveryMs)
                            + "ms at " + (int) multiVelocityRPMs[multiVelocityIndex] + " RPM");
                } else {
                    publishStatus("Phase 3: PASS - " + (int) multiVelocityRPMs[multiVelocityIndex]
                            + " RPM recovery " + String.format("%.0f", recoveryMs) + "ms");
                }

                multiVelocityIndex++;
                subStep = 3;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== PHASE 4: VALIDATION =====

    private void initValidation() {
        validationTests = new double[][] {
            { 0, 2000 },
            { 2000, 4000 },
            { 4000, 2000 },
            { 2000, 5000 },
            { 5000, 3000 }
        };
        validationIndex = 0;
        validationDisturbanceDone = false;
        publishStatus("Phase 4: Validation - running " + (validationTests.length + 1) + " tests");
    }

    private void executeValidation() {
        SmartDashboard.putString(PREFIX + "Phase", "Validation");

        switch (subStep) {
        case 0: // Set start velocity and settle
            if (validationIndex < validationTests.length) {
                double startRPM = validationTests[validationIndex][0];
                if (startRPM == 0) {
                    flywheel.stop();
                } else {
                    flywheel.setVelocity(startRPM);
                }
            } else if (!validationDisturbanceDone) {
                flywheel.setVelocity(3500);
            } else {
                publishResults();
                currentPhase = Phase.DONE;
                break;
            }
            if (cycleCount > (int)(SETTLE_SECONDS * 50)) {
                if (validationIndex < validationTests.length) {
                    double startRPS = validationTests[validationIndex][0] / 60.0;
                    double endRPS = validationTests[validationIndex][1] / 60.0;
                    subStep = 1;
                    cycleCount = 0;
                    startVelocityStepTest(startRPS, endRPS);
                    publishStatus("Phase 4: Test " + (validationIndex + 1) + "/"
                            + (validationTests.length + 1) + " → "
                            + (int) validationTests[validationIndex][0] + "→"
                            + (int) validationTests[validationIndex][1] + " RPM");
                } else {
                    // Disturbance validation test
                    subStep = 2;
                    cycleCount = 0;
                    disturbancePulseActive = false;
                    analyzer = new VelocityStepResponseAnalyzer(
                            3500.0 / 60.0, 3500.0 / 60.0, VELOCITY_TOLERANCE_RPS);
                    publishStatus("Phase 4: Test " + (validationTests.length + 1) + "/"
                            + (validationTests.length + 1) + " → Disturbance at 3500 RPM");
                }
            }
            break;

        case 1: // Velocity step test
            if (runVelocityStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                double riseTime = analyzer.getRiseTime();
                double ssError = analyzer.getSteadyStateError();
                publishStatus("Phase 4: " + (int) validationTests[validationIndex][0] + "→"
                        + (int) validationTests[validationIndex][1] + " RPM → "
                        + "rise=" + fmt(riseTime) + "s overshoot=" + String.format("%.1f", overshoot) + "% "
                        + "settling=" + fmt(settlingTime) + "s error=" + fmt(ssError * 60) + "RPM");
                validationIndex++;
                subStep = 0;
                cycleCount = 0;
            }
            break;

        case 2: // Disturbance validation test
            double nowVal = System.nanoTime() / 1e9;
            analyzer.addSample(flywheel.getVelocityRPS());

            if (!disturbancePulseActive && cycleCount > 25) {
                if (flywheel.getCurrentVelocityRPM() > MIN_RPM_FOR_OPPOSING_CURRENT) {
                    disturbancePulseActive = true;
                    disturbancePulseStart = nowVal;
                    flywheel.applyCurrent(DISTURBANCE_CURRENT);
                }
            }

            if (disturbancePulseActive) {
                double pulseElapsedMs = (nowVal - disturbancePulseStart) * 1000.0;
                if (pulseElapsedMs >= DISTURBANCE_DURATION_MS) {
                    flywheel.setVelocity(3500);
                    disturbancePulseActive = false;
                }
            }

            if (cycleCount > 25 + (int)(2.0 * 50)) {
                double disturbanceTime = 25.0 / 50.0;
                double recoveryMs = analyzer.getRecoveryTime(disturbanceTime) * 1000.0;
                publishStatus("Phase 4: Disturbance at 3500 RPM → recovery=" + String.format("%.0f", recoveryMs) + "ms");
                validationDisturbanceDone = true;
                subStep = 0;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== STEP TEST HELPERS =====

    private void startVelocityStepTest(double fromRPS, double toRPS) {
        double currentVel = flywheel.getVelocityRPS();
        double tolerance = Math.max(VELOCITY_TOLERANCE_RPS, Math.abs(toRPS - fromRPS) * 0.05);
        analyzer = new VelocityStepResponseAnalyzer(currentVel, toRPS, tolerance);
        settling = false;
        stepStartTime = System.nanoTime() / 1e9;

        // Command the step
        flywheel.setVelocity(toRPS * 60.0); // convert RPS to RPM
    }

    /**
     * Runs the velocity step test recording. Returns true when the test is complete.
     */
    private boolean runVelocityStepTest() {
        double now = System.nanoTime() / 1e9;
        double elapsed = now - stepStartTime;

        analyzer.addSample(flywheel.getVelocityRPS());

        SmartDashboard.putNumber(PREFIX + "Target RPS", analyzer.getTargetVelocity());
        SmartDashboard.putNumber(PREFIX + "Current RPS", flywheel.getVelocityRPS());

        return elapsed >= STEP_RECORD_SECONDS;
    }

    // ===== RESULTS =====

    private void publishResults() {
        System.out.println("\n====================================");
        System.out.println("   FLYWHEEL AUTO-TUNE RESULTS");
        System.out.println("====================================");
        System.out.println("// Paste into Constants.java FlywheelConstants:");
        System.out.println("public static final double S = " + fmt(tunedKs) + ";");
        System.out.println("public static final double V = " + fmt(tunedKv) + ";");
        System.out.println("public static final double A = " + fmt(tunedKa) + ";");
        System.out.println("public static final double P = " + fmt(tunedKp) + ";");
        System.out.println("public static final double I = " + fmt(tunedKi) + ";");
        System.out.println("public static final double D = " + fmt(tunedKd) + ";");
        System.out.println("====================================\n");

        SmartDashboard.putNumber(PREFIX + "Results/kS", tunedKs);
        SmartDashboard.putNumber(PREFIX + "Results/kV", tunedKv);
        SmartDashboard.putNumber(PREFIX + "Results/kA", tunedKa);
        SmartDashboard.putNumber(PREFIX + "Results/kP", tunedKp);
        SmartDashboard.putNumber(PREFIX + "Results/kI", tunedKi);
        SmartDashboard.putNumber(PREFIX + "Results/kD", tunedKd);

        publishStatus("COMPLETE - Tuned gains are active. See console for copy-paste values.");
        SmartDashboard.putNumber(PREFIX + "Progress", 1.0);
        System.out.println("=== FLYWHEEL AUTO-TUNE COMPLETE - Gains are active for testing ===");
    }

    // ===== LIFECYCLE =====

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            flywheel.restoreDefaultConfig();
            flywheel.stop();
            publishStatus("CANCELLED - Original gains restored");
            System.out.println("=== FLYWHEEL AUTO-TUNE CANCELLED - Original gains restored ===");
        } else {
            flywheel.stop();
        }
    }

    // ===== UTILITIES =====

    private double getProgress() {
        switch (currentPhase) {
        case FEEDFORWARD:
            return subStep / 20.0; // ~0 to 0.25
        case PID_TUNING:
            return 0.25 + (subStep / 28.0); // ~0.25 to 0.50
        case DISTURBANCE_REJECTION:
            return 0.50 + (subStep / 20.0); // ~0.50 to 0.75
        case VALIDATION:
            int totalTests = validationTests != null ? validationTests.length + 1 : 6;
            return 0.75 + ((double) validationIndex / totalTests) * 0.25;
        case DONE:
            return 1.0;
        default:
            return 0;
        }
    }

    private void publishStatus(String status) {
        SmartDashboard.putString(PREFIX + "Status", status);
        System.out.println("[FlywheelAutoTune] " + status);
    }

    private static String fmt(double v) {
        return String.format("%.4f", v);
    }
}
