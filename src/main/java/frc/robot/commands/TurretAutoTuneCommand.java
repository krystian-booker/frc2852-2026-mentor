package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.StepResponseAnalyzer;

/**
 * Automated PID tuning command for the turret. Runs as a 4-phase state machine:
 * <ol>
 * <li>Feedforward characterization (kS, kV, kA)</li>
 * <li>PID tuning via step-response binary search</li>
 * <li>Motion Magic profile optimization</li>
 * <li>Validation and report</li>
 * </ol>
 * Total runtime ~3.5 minutes. On interruption, restores original gains.
 */
public class TurretAutoTuneCommand extends Command {

    private static final double SAFE_POSITION_LIMIT = 360.0; // degrees, 1 full rotation - catches runaway, not normal
                                                             // overshoot
    private static final double HARD_POSITION_LIMIT = 720.0; // degrees, 2 full rotations - immediate abort
    private static final int MAX_SAFETY_VIOLATIONS = 100; // 2s at 50Hz before abort
    private static final double MAX_CHARACTERIZATION_VOLTAGE = 4.0;
    private static final double KS_ABORT_VOLTAGE = 3.0;
    private static final double KS_RAMP_RATE = 0.005; // V per cycle (~0.25 V/s at 50Hz)
    private static final double KS_VELOCITY_THRESHOLD = 0.005; // RPS
    private static final int KS_CONSECUTIVE_SAMPLES = 3;
    private static final double SETTLE_TIME_SECONDS = 2.0;
    private static final double STEP_RECORD_SECONDS = 3.0;
    private static final double POSITION_TOLERANCE_ROTATIONS = 1.0 / 360.0; // 1 degree in rotations
    private static final String PREFIX = "AutoTune/";

    private final Turret turret;

    // State machine
    private enum Phase {
        FEEDFORWARD, PID_TUNING, MOTION_MAGIC, VALIDATION, DONE
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
    private double tunedKd;

    // Motion Magic results
    private double tunedCruise;
    private double tunedAccel;
    private double tunedJerk;

    // Phase 1 (FF) working state
    private double ksRampVoltage;
    private int ksConsecutiveCount;
    private double ksPositive;
    private double ksNegative;
    private boolean ksPositiveDone;
    private double[] kvTestVoltages;
    private double[] kvMeasuredVelocities;
    private int kvTestIndex;
    private int kvDirection = 1; // alternates +1/-1 for kV tests
    private double kvSettleStart;
    private double kvAccumVelocity;
    private int kvAccumCount;
    private double kaStepStart;
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
    private double bestKdSettlingTime;
    private double bestKdOvershoot;
    private double bestKd;
    private int validationStepIndex;
    private double[] validationStepSizes;

    // Phase 3 (Motion Magic) working state
    private double[] cruiseCandidates;
    private int cruiseTestIndex;
    private double[] accelCandidates;
    private int accelTestIndex;

    // Phase 4 (Validation) working state
    private double[] validationTargets;
    private int validationIndex;

    // Safety
    private int consecutiveSafetyViolations;

    // Step response recording
    private StepResponseAnalyzer analyzer;
    private double stepStartTime;
    private boolean settling;
    private boolean recording;
    private double stepTargetPosition; // in degrees

    public TurretAutoTuneCommand(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        System.out.println("=== TURRET AUTO-TUNE STARTING ===");
        publishStatus("Starting...");

        currentPhase = Phase.FEEDFORWARD;
        subStep = 0;
        cycleCount = 0;

        // Initialize feedforward state
        ksRampVoltage = 0;
        ksConsecutiveCount = 0;
        ksPositive = 0;
        ksNegative = 0;
        ksPositiveDone = false;
        kvDirection = 1;

        tunedKs = 0;
        tunedKv = 0;
        tunedKa = 0.01;
        tunedKp = 0;
        tunedKd = 0;
        tunedCruise = 2.5;
        tunedAccel = 10.0;
        tunedJerk = 200.0;

        consecutiveSafetyViolations = 0;

        // Center the turret first
        turret.setPosition(0);
    }

    @Override
    public void execute() {
        cycleCount++;

        // Publish progress telemetry
        double currentPos = turret.getPositionDegrees();
        SmartDashboard.putNumber(PREFIX + "Position", currentPos);
        SmartDashboard.putNumber(PREFIX + "Velocity", turret.getVelocityRPS());

        switch (currentPhase) {
        case FEEDFORWARD:
            executeFeedforward();
            break;
        case PID_TUNING:
            executePidTuning();
            break;
        case MOTION_MAGIC:
            executeMotionMagic();
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
        case 0: // Wait for turret to center
            if (cycleCount > 100) { // 2 seconds at 50Hz
                turret.stop();
                subStep = 1;
                ksRampVoltage = 0;
                ksConsecutiveCount = 0;
                publishStatus("Phase 1: Finding kS (positive direction)");
            }
            break;

        case 1: // kS ramp positive
            ksRampVoltage += KS_RAMP_RATE;
            if (ksRampVoltage > KS_ABORT_VOLTAGE) {
                System.err.println("AUTO-TUNE: kS ramp exceeded " + KS_ABORT_VOLTAGE + "V without motion. Aborting.");
                cancel();
                return;
            }
            turret.applyVoltage(ksRampVoltage);
            if (Math.abs(turret.getVelocityRPS()) > KS_VELOCITY_THRESHOLD) {
                ksConsecutiveCount++;
            } else {
                ksConsecutiveCount = 0;
            }
            if (ksConsecutiveCount >= KS_CONSECUTIVE_SAMPLES) {
                ksPositive = ksRampVoltage;
                ksPositiveDone = true;
                turret.applyVoltage(0);
                ksRampVoltage = 0;
                ksConsecutiveCount = 0;
                subStep = 2;
                cycleCount = 0;
                publishStatus("Phase 1: Settling before kS negative");
            }
            break;

        case 2: // Brief settle before negative direction
            turret.setPosition(0);
            if (cycleCount > 50) {
                subStep = 3;
                publishStatus("Phase 1: Finding kS (negative direction)");
            }
            break;

        case 3: // kS ramp negative
            ksRampVoltage += KS_RAMP_RATE;
            if (ksRampVoltage > KS_ABORT_VOLTAGE) {
                System.err.println(
                        "AUTO-TUNE: kS ramp (negative) exceeded " + KS_ABORT_VOLTAGE + "V without motion. Aborting.");
                cancel();
                return;
            }
            turret.applyVoltage(-ksRampVoltage);
            if (Math.abs(turret.getVelocityRPS()) > KS_VELOCITY_THRESHOLD) {
                ksConsecutiveCount++;
            } else {
                ksConsecutiveCount = 0;
            }
            if (ksConsecutiveCount >= KS_CONSECUTIVE_SAMPLES) {
                ksNegative = ksRampVoltage;
                tunedKs = (ksPositive + ksNegative) / 2.0;
                turret.applyVoltage(0);
                subStep = 4;
                cycleCount = 0;
                publishStatus("Phase 1: kS = " + String.format("%.3f", tunedKs) + " - preparing kV tests");
                SmartDashboard.putNumber(PREFIX + "kS", tunedKs);

                // Prepare kV test voltages
                kvTestVoltages = new double[] { tunedKs + 1, tunedKs + 2, tunedKs + 3, tunedKs + 4 };
                kvMeasuredVelocities = new double[4];
                kvTestIndex = 0;
            }
            break;

        case 4: // Settle before kV tests - position at opposite extreme for maximum travel
            double kvStartPos = -170.0 * kvDirection; // opposite side from voltage direction
            turret.setPosition(kvStartPos);
            if (cycleCount > 100) { // 2s to reach extreme
                subStep = 5;
                cycleCount = 0;
                kvSettleStart = -1;
                kvAccumVelocity = 0;
                kvAccumCount = 0;
                publishStatus("Phase 1: kV test " + (kvTestIndex + 1) + "/4 at " +
                        String.format("%.1f", kvTestVoltages[kvTestIndex]) + "V");
            }
            break;

        case 5: // kV test: apply voltage, settle, measure
            double testV = Math.min(kvTestVoltages[kvTestIndex], MAX_CHARACTERIZATION_VOLTAGE);
            turret.applyVoltage(testV * kvDirection);
            double elapsed = cycleCount / 50.0;

            boolean approachingLimit = Math.abs(turret.getPositionDegrees()) > (SAFE_POSITION_LIMIT - 10);

            if (elapsed < SETTLE_TIME_SECONDS) {
                // Settling phase
            } else if (elapsed < SETTLE_TIME_SECONDS + 0.5) {
                // Measure phase (0.5s of samples)
                kvAccumVelocity += Math.abs(turret.getVelocityRPS());
                kvAccumCount++;
                // Early termination if approaching limit with enough samples
                if (approachingLimit && kvAccumCount > 5) {
                    System.out.println("[AutoTune] kV test " + (kvTestIndex + 1) +
                            " ending early - approaching position limit at " +
                            String.format("%.1f", turret.getPositionDegrees()) + "°");
                    elapsed = SETTLE_TIME_SECONDS + 0.5; // force into completion branch
                }
            }
            if (elapsed >= SETTLE_TIME_SECONDS + 0.5) {
                // Done with this voltage
                double avgVel = kvAccumVelocity / kvAccumCount;
                kvMeasuredVelocities[kvTestIndex] = avgVel;
                turret.applyVoltage(0);
                kvTestIndex++;
                kvDirection *= -1; // alternate direction for next test

                if (kvTestIndex < 4) {
                    subStep = 4; // settle before next test
                    cycleCount = 0;
                } else {
                    // Compute kV from all measurements
                    double kvSum = 0;
                    int kvValid = 0;
                    for (int i = 0; i < 4; i++) {
                        if (kvMeasuredVelocities[i] > 0.01) {
                            double actualVoltage = Math.min(kvTestVoltages[i], MAX_CHARACTERIZATION_VOLTAGE);
                            kvSum += (actualVoltage - tunedKs) / kvMeasuredVelocities[i];
                            kvValid++;
                        }
                    }
                    tunedKv = kvValid > 0 ? kvSum / kvValid : 5.0;
                    SmartDashboard.putNumber(PREFIX + "kV", tunedKv);
                    publishStatus("Phase 1: kV = " + String.format("%.3f", tunedKv) + " - measuring kA");
                    subStep = 6;
                    cycleCount = 0;
                }
                kvAccumVelocity = 0;
                kvAccumCount = 0;
            }
            break;

        case 6: // Settle before kA test - position at extreme for maximum travel
            turret.setPosition(170.0); // start at extreme, kA drives toward center
            if (cycleCount > 100) { // 2s to reach extreme
                subStep = 7;
                cycleCount = 0;
                kaPrevVelocity = 0;
                kaAccelSum = 0;
                kaAccelCount = 0;
            }
            break;

        case 7: // kA: step voltage, measure initial acceleration
            double kaVoltage = Math.min(tunedKs + 2.0, MAX_CHARACTERIZATION_VOLTAGE);
            double kaDirection = turret.getPositionDegrees() > 0 ? -1.0 : 1.0; // drive toward center
            turret.applyVoltage(kaVoltage * kaDirection);
            double kaElapsed = cycleCount / 50.0;

            if (kaElapsed > 0.02 && kaElapsed < 0.3) {
                double vel = Math.abs(turret.getVelocityRPS());
                if (kaPrevVelocity > 0) {
                    double accel = (vel - kaPrevVelocity) * 50.0; // dv/dt at 50Hz
                    if (accel > 0) {
                        kaAccelSum += accel;
                        kaAccelCount++;
                    }
                }
                kaPrevVelocity = vel;
            } else if (kaElapsed >= 0.3) {
                turret.applyVoltage(0);
                if (kaAccelCount > 0) {
                    double avgAccel = kaAccelSum / kaAccelCount;
                    if (avgAccel > 0.1) {
                        tunedKa = (kaVoltage - tunedKs) / avgAccel;
                    }
                }
                // Clamp kA to reasonable range
                tunedKa = Math.max(0.001, Math.min(0.1, tunedKa));
                SmartDashboard.putNumber(PREFIX + "kA", tunedKa);
                publishStatus("Phase 1 COMPLETE: kS=" + fmt(tunedKs) + " kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));
                System.out
                        .println("Phase 1 Results: kS=" + fmt(tunedKs) + " kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));

                // Move to center and transition to PID tuning
                turret.setPosition(0);
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
        // Apply characterized feedforward with initial PID
        kpTest = 10.0;
        kpLow = 0;
        kpHigh = 0;
        binarySearchIteration = 0;
        kpSearchDone = false;
        tunedKp = 10.0;
        tunedKd = 0;

        kdCandidates = null;
        kdTestIndex = 0;
        bestKdSettlingTime = Double.MAX_VALUE;
        bestKd = 0;

        validationStepSizes = new double[] { 30, 60, 90, 120 };
        validationStepIndex = 0;

        // Apply feedforward gains with starting PID
        turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, kpTest, 0, 0,
                tunedCruise, tunedAccel, tunedJerk);
        publishStatus("Phase 2: PID tuning - starting kP search at " + kpTest);
    }

    private void executePidTuning() {
        SmartDashboard.putString(PREFIX + "Phase", "PID Tuning");

        switch (subStep) {
        case 0: // Wait for center settle
            if (cycleCount > 75) {
                subStep = 1;
                cycleCount = 0;
                startStepTest(0, 90); // 90° step from center
            }
            break;

        case 1: // kP doubling / binary search: run step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                SmartDashboard.putNumber(PREFIX + "StepResponse/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "StepResponse/RiseTime", analyzer.getRiseTime());
                SmartDashboard.putNumber(PREFIX + "StepResponse/SettlingTime", settlingTime);
                publishStatus("Phase 2: kP=" + fmt(kpTest) + " overshoot=" + String.format("%.1f", overshoot) + "%");

                if (!kpSearchDone) {
                    if (overshoot > 15.0) {
                        // Too much overshoot - start binary search
                        kpHigh = kpTest;
                        kpLow = 0;
                        kpSearchDone = true; // switch to binary search mode
                        binarySearchIteration = 0;
                    } else {
                        // Not enough overshoot, keep doubling
                        if (kpTest >= 80) {
                            // Hit ceiling, use this value
                            tunedKp = kpTest;
                            finishKpSearch();
                            break;
                        }
                        kpTest *= 2;
                    }
                }

                if (kpSearchDone) {
                    // Update bounds based on overshoot result
                    if (overshoot > 15.0) {
                        kpHigh = kpTest;
                    } else {
                        kpLow = kpTest;
                    }

                    if (binarySearchIteration >= 6) {
                        tunedKp = kpLow; // use last known good
                        finishKpSearch();
                        break;
                    }
                    kpTest = (kpLow + kpHigh) / 2.0;
                    binarySearchIteration++;
                }

                // Apply new kP and run another step test
                turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, kpTest, 0, 0,
                        tunedCruise, tunedAccel, tunedJerk);
                // Return to center
                turret.setPosition(0);
                subStep = 2;
                cycleCount = 0;
            }
            break;

        case 2: // Settle at center before next kP test
            if (cycleCount > 75) {
                subStep = 1;
                cycleCount = 0;
                startStepTest(0, 90);
            }
            break;

        case 3: // kD search: run step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                SmartDashboard.putNumber(PREFIX + "StepResponse/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "StepResponse/SettlingTime", settlingTime);

                if (overshoot < 20.0 && overshoot < bestKdOvershoot) {
                    bestKdOvershoot = overshoot;
                    bestKd = kdCandidates[kdTestIndex];
                }
                publishStatus("Phase 2: kD=" + fmt(kdCandidates[kdTestIndex]) +
                        " overshoot=" + String.format("%.1f", overshoot) + "% settling=" + fmt(settlingTime) + "s");

                kdTestIndex++;
                if (kdTestIndex >= kdCandidates.length) {
                    tunedKd = bestKd;
                    SmartDashboard.putNumber(PREFIX + "kP", tunedKp);
                    SmartDashboard.putNumber(PREFIX + "kD", tunedKd);
                    publishStatus("Phase 2: kP=" + fmt(tunedKp) + " kD=" + fmt(tunedKd) + " - validating");

                    // Apply final PID for validation
                    turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, tunedKd,
                            tunedCruise, tunedAccel, tunedJerk);
                    turret.setPosition(0);
                    subStep = 5;
                    cycleCount = 0;
                    validationStepIndex = 0;
                } else {
                    // Test next kD
                    turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, kdCandidates[kdTestIndex],
                            tunedCruise, tunedAccel, tunedJerk);
                    turret.setPosition(0);
                    subStep = 4;
                    cycleCount = 0;
                }
            }
            break;

        case 4: // Settle at center before next kD test
            if (cycleCount > 75) {
                subStep = 3;
                cycleCount = 0;
                startStepTest(0, 90);
            }
            break;

        case 5: // Settle before PID validation steps
            if (cycleCount > 75) {
                if (validationStepIndex < validationStepSizes.length) {
                    subStep = 6;
                    cycleCount = 0;
                    startStepTest(0, validationStepSizes[validationStepIndex]);
                } else {
                    // PID validation done, move to Motion Magic
                    publishStatus("Phase 2 COMPLETE: kP=" + fmt(tunedKp) + " kD=" + fmt(tunedKd));
                    System.out.println("Phase 2 Results: kP=" + fmt(tunedKp) + " kD=" + fmt(tunedKd));
                    currentPhase = Phase.MOTION_MAGIC;
                    subStep = 0;
                    cycleCount = 0;
                    initMotionMagic();
                }
            }
            break;

        case 6: // PID validation step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                publishStatus("Phase 2 validation: " + validationStepSizes[validationStepIndex] +
                        "° overshoot=" + String.format("%.1f", overshoot) + "% settling=" + fmt(settlingTime) + "s");
                validationStepIndex++;
                turret.setPosition(0);
                subStep = 5;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    private void finishKpSearch() {
        SmartDashboard.putNumber(PREFIX + "kP", tunedKp);
        publishStatus("Phase 2: kP=" + fmt(tunedKp) + " - starting kD search");

        // Setup kD candidates
        kdCandidates = new double[] { 0, tunedKp * 0.01, tunedKp * 0.02, tunedKp * 0.05 };
        kdTestIndex = 0;
        bestKdSettlingTime = Double.MAX_VALUE;
        bestKdOvershoot = Double.MAX_VALUE;
        bestKd = 0;

        // Apply first kD candidate
        turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, kdCandidates[0],
                tunedCruise, tunedAccel, tunedJerk);
        turret.setPosition(0);
        subStep = 4;
        cycleCount = 0;
    }

    // ===== PHASE 3: MOTION MAGIC OPTIMIZATION =====

    private void initMotionMagic() {
        cruiseCandidates = new double[] { 1, 2, 3, 4, 5 };
        cruiseTestIndex = 0;
        accelCandidates = new double[] { 5, 10, 15, 20, 25 };
        accelTestIndex = 0;
        tunedCruise = 2.5; // default fallback
        tunedAccel = 10.0;
        publishStatus("Phase 3: Motion Magic optimization - testing cruise velocities");
    }

    private void executeMotionMagic() {
        SmartDashboard.putString(PREFIX + "Phase", "Motion Magic");

        switch (subStep) {
        case 0: // Settle at center before cruise velocity test
            turret.setPosition(0);
            if (cycleCount > 75) {
                if (cruiseTestIndex < cruiseCandidates.length) {
                    turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, tunedKd,
                            cruiseCandidates[cruiseTestIndex], tunedAccel, tunedAccel * 20);
                    subStep = 1;
                    cycleCount = 0;
                    startStepTest(0, 45);
                    publishStatus("Phase 3: cruise=" + fmt(cruiseCandidates[cruiseTestIndex]) + " RPS");
                } else {
                    // All cruise tests done, pick best
                    // tunedCruise is already set by the selection logic
                    SmartDashboard.putNumber(PREFIX + "CruiseVelocity", tunedCruise);
                    publishStatus("Phase 3: cruise=" + fmt(tunedCruise) + " - testing accelerations");
                    accelTestIndex = 0;
                    subStep = 2;
                    cycleCount = 0;
                }
            }
            break;

        case 1: // Cruise velocity step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                publishStatus("Phase 3: cruise=" + fmt(cruiseCandidates[cruiseTestIndex]) +
                        " overshoot=" + String.format("%.1f", overshoot) + "% settling=" + fmt(settlingTime) + "s");

                if (overshoot < 10.0) {
                    tunedCruise = cruiseCandidates[cruiseTestIndex]; // pick fastest that works
                }
                cruiseTestIndex++;
                turret.setPosition(0);
                subStep = 0;
                cycleCount = 0;
            }
            break;

        case 2: // Settle before acceleration test
            turret.setPosition(0);
            if (cycleCount > 75) {
                if (accelTestIndex < accelCandidates.length) {
                    turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, tunedKd,
                            tunedCruise, accelCandidates[accelTestIndex], accelCandidates[accelTestIndex] * 20);
                    subStep = 3;
                    cycleCount = 0;
                    startStepTest(0, 45);
                    publishStatus("Phase 3: accel=" + fmt(accelCandidates[accelTestIndex]) + " RPS/s");
                } else {
                    // All accel tests done
                    tunedJerk = tunedAccel * 20;
                    SmartDashboard.putNumber(PREFIX + "Acceleration", tunedAccel);
                    publishStatus("Phase 3 COMPLETE: cruise=" + fmt(tunedCruise) +
                            " accel=" + fmt(tunedAccel) + " jerk=" + fmt(tunedJerk));
                    System.out.println("Phase 3 Results: cruise=" + fmt(tunedCruise) +
                            " accel=" + fmt(tunedAccel) + " jerk=" + fmt(tunedJerk));

                    // Apply final config and move to validation
                    turret.applyTuningConfig(tunedKs, tunedKv, tunedKa, 0, tunedKp, 0, tunedKd,
                            tunedCruise, tunedAccel, tunedJerk);
                    currentPhase = Phase.VALIDATION;
                    subStep = 0;
                    cycleCount = 0;
                    initValidation();
                }
            }
            break;

        case 3: // Acceleration step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                publishStatus("Phase 3: accel=" + fmt(accelCandidates[accelTestIndex]) +
                        " overshoot=" + String.format("%.1f", overshoot) + "% settling=" + fmt(settlingTime) + "s");

                if (overshoot < 10.0) {
                    tunedAccel = accelCandidates[accelTestIndex]; // pick fastest that works
                }
                accelTestIndex++;
                turret.setPosition(0);
                subStep = 2;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== PHASE 4: VALIDATION =====

    private void initValidation() {
        validationTargets = new double[] { 45, -90, 120, -60, 90 };
        validationIndex = 0;
        publishStatus("Phase 4: Validation - running " + validationTargets.length + " test moves");
    }

    private void executeValidation() {
        SmartDashboard.putString(PREFIX + "Phase", "Validation");

        switch (subStep) {
        case 0: // Settle at center
            turret.setPosition(0);
            if (cycleCount > 75) {
                if (validationIndex < validationTargets.length) {
                    subStep = 1;
                    cycleCount = 0;
                    startStepTest(0, validationTargets[validationIndex]);
                    publishStatus("Phase 4: Test " + (validationIndex + 1) + "/" +
                            validationTargets.length + " → " + validationTargets[validationIndex] + "°");
                } else {
                    // All validation done
                    publishResults();
                    currentPhase = Phase.DONE;
                }
            }
            break;

        case 1: // Validation step test
            if (runStepTest()) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                double ssError = analyzer.getSteadyStateError();
                publishStatus("Phase 4: " + validationTargets[validationIndex] + "° → " +
                        "overshoot=" + String.format("%.1f", overshoot) + "% settling=" +
                        fmt(settlingTime) + "s error=" + fmt(ssError * 360.0) + "°");
                validationIndex++;
                turret.setPosition(0);
                subStep = 0;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== STEP TEST HELPERS =====

    private void startStepTest(double fromDegrees, double toDegrees) {
        stepTargetPosition = toDegrees;
        // Clamp target within safe limits
        stepTargetPosition = Math.max(-SAFE_POSITION_LIMIT, Math.min(SAFE_POSITION_LIMIT, stepTargetPosition));

        double currentPos = turret.getPositionDegrees();
        double stepSizeRotations = Math.abs(stepTargetPosition / 360.0 - currentPos / 360.0);
        double tolerance = Math.max(2.0 / 360.0, stepSizeRotations * 0.05);
        analyzer = new StepResponseAnalyzer(
                currentPos / 360.0,
                stepTargetPosition / 360.0,
                tolerance);
        settling = true;
        recording = false;
        stepStartTime = System.nanoTime() / 1e9;

        // Command the step
        turret.setPosition(stepTargetPosition);
    }

    /**
     * Runs the step test recording. Returns true when the test is complete.
     */
    private boolean runStepTest() {
        double now = System.nanoTime() / 1e9;
        double elapsed = now - stepStartTime;
        double posRotations = turret.getPositionDegrees() / 360.0;

        // Record samples for the full duration
        analyzer.addSample(posRotations);

        // Total test time = STEP_RECORD_SECONDS
        return elapsed >= STEP_RECORD_SECONDS;
    }

    // ===== RESULTS =====

    private void publishResults() {
        System.out.println("\n====================================");
        System.out.println("   TURRET AUTO-TUNE RESULTS");
        System.out.println("====================================");
        System.out.println("// Paste into Constants.java TurretConstants:");
        System.out.println("public static final double S = " + fmt(tunedKs) + ";");
        System.out.println("public static final double V = " + fmt(tunedKv) + ";");
        System.out.println("public static final double A = " + fmt(tunedKa) + ";");
        System.out.println("public static final double G = 0.0;");
        System.out.println("public static final double P = " + fmt(tunedKp) + ";");
        System.out.println("public static final double I = 0.0;");
        System.out.println("public static final double D = " + fmt(tunedKd) + ";");
        System.out.println("public static final double MOTION_MAGIC_CRUISE_VELOCITY = " + fmt(tunedCruise) + ";");
        System.out.println("public static final double MOTION_MAGIC_ACCELERATION = " + fmt(tunedAccel) + ";");
        System.out.println("public static final double MOTION_MAGIC_JERK = " + fmt(tunedJerk) + ";");
        System.out.println("====================================\n");

        SmartDashboard.putNumber(PREFIX + "Results/kS", tunedKs);
        SmartDashboard.putNumber(PREFIX + "Results/kV", tunedKv);
        SmartDashboard.putNumber(PREFIX + "Results/kA", tunedKa);
        SmartDashboard.putNumber(PREFIX + "Results/kP", tunedKp);
        SmartDashboard.putNumber(PREFIX + "Results/kD", tunedKd);
        SmartDashboard.putNumber(PREFIX + "Results/CruiseVelocity", tunedCruise);
        SmartDashboard.putNumber(PREFIX + "Results/Acceleration", tunedAccel);
        SmartDashboard.putNumber(PREFIX + "Results/Jerk", tunedJerk);

        publishStatus("COMPLETE - Tuned gains are active. See console for copy-paste values.");
        SmartDashboard.putNumber(PREFIX + "Progress", 1.0);
        System.out.println("=== TURRET AUTO-TUNE COMPLETE - Gains are active for testing ===");
    }

    // ===== LIFECYCLE =====

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Restore original Constants.java values
            turret.restoreDefaultConfig();
            turret.stop();
            publishStatus("CANCELLED - Original gains restored");
            System.out.println("=== TURRET AUTO-TUNE CANCELLED - Original gains restored ===");
        } else {
            // Leave tuned gains active so operator can test
            turret.stop();
        }
    }

    // ===== UTILITIES =====

    private double getProgress() {
        switch (currentPhase) {
        case FEEDFORWARD:
            return subStep / 32.0; // ~0 to 0.25
        case PID_TUNING:
            return 0.25 + (subStep / 28.0); // ~0.25 to 0.50
        case MOTION_MAGIC:
            return 0.50 + (subStep / 16.0); // ~0.50 to 0.75
        case VALIDATION:
            return 0.75 + ((double) validationIndex / validationTargets.length) * 0.25;
        case DONE:
            return 1.0;
        default:
            return 0;
        }
    }

    private void publishStatus(String status) {
        SmartDashboard.putString(PREFIX + "Status", status);
        System.out.println("[AutoTune] " + status);
    }

    private static String fmt(double v) {
        return String.format("%.4f", v);
    }
}
