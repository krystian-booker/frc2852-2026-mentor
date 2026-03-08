package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;
import frc.robot.util.StepResponseAnalyzer;

/**
 * Automated feedforward tuning command for the Hood arm mechanism.
 * Characterizes kG, kS, kV, kA while leaving PID/Motion Magic unchanged.
 *
 * <p>Key insight: With 258:1 gearing, gearbox friction dominates gravity.
 * The hood does NOT freefall when the motor is neutral. To separate gravity
 * from friction, we use bidirectional voltage ramps at each test angle:
 * <ul>
 * <li>Ramp positive until motion → V_up = V_gravity + V_friction</li>
 * <li>Ramp negative until motion → V_down = V_gravity - V_friction</li>
 * <li>V_gravity = (V_up + V_down) / 2</li>
 * <li>V_friction = (V_up - V_down) / 2</li>
 * </ul>
 *
 * <ol>
 * <li>kG + kS characterization via bidirectional ramps at 2 angles</li>
 * <li>kV/kA characterization</li>
 * <li>Validation step tests with all FF + existing PID</li>
 * </ol>
 * Total runtime ~1.5-2 minutes. On interruption, restores original gains.
 */
public class HoodAutoTuneCommand extends Command {

    // Safety limits
    private static final double MAX_CHARACTERIZATION_VOLTAGE = 2.0;
    private static final double POSITION_MIN_ABORT = -1.0; // degrees
    private static final double POSITION_MAX_ABORT = 26.0; // degrees
    private static final double VELOCITY_ABORT_RPS = 0.5; // mechanism RPS
    private static final double CURRENT_ABORT_AMPS = 15.0;
    private static final int CURRENT_ABORT_CONSECUTIVE = 50;

    // Characterization parameters
    private static final double RAMP_RATE = 0.005; // V per cycle (~0.25 V/s at 50Hz)
    private static final double MOTION_VELOCITY_THRESHOLD = 0.003; // RPS - detect motion start
    private static final int MOTION_CONSECUTIVE_SAMPLES = 5;
    private static final double RAMP_ABORT_VOLTAGE = 1.5;
    private static final int SETTLE_CYCLES = 100; // 2s at 50Hz
    private static final double STEP_RECORD_SECONDS = 3.0;
    private static final String PREFIX = "HoodAutoTune/";

    private final Hood hood;

    // State machine
    private enum Phase {
        KG_KS_CHARACTERIZATION, KV_KA_CHARACTERIZATION, VALIDATION, DONE
    }

    private Phase currentPhase;
    private int subStep;
    private int cycleCount;
    private boolean aborted;

    // Safety
    private int consecutiveCurrentViolations;

    // kG/kS bidirectional ramp working state
    private double rampVoltage;
    private int motionConsecutiveCount;

    // Results from angle 1 (12.5 deg)
    private double angle1Deg;
    private double vUp1;   // voltage where positive motion starts
    private double vDown1; // voltage where negative motion starts (negative value)

    // Results from angle 2 (20 deg)
    private double angle2Deg;
    private double vUp2;
    private double vDown2;

    // kG verification
    private double kgVerifyStartPos;

    // kV/kA working state
    private double[] kvTestVoltages;
    private double[] kvMeasuredVelocities;
    private int kvTestIndex;
    private int kvMeasureCount;
    private double kvVelocityAccum;
    private double kaPrevVelocity;
    private double kaAccelSum;
    private int kaAccelCount;

    // Tuned results
    private double tunedKg;
    private double tunedKs;
    private double tunedKv;
    private double tunedKa;

    // Validation
    private double[] validationTargets;
    private int validationIndex;
    private StepResponseAnalyzer analyzer;
    private double stepStartTime;

    public HoodAutoTuneCommand(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        System.out.println("=== HOOD FF AUTO-TUNE STARTING ===");
        publishStatus("Starting...");

        currentPhase = Phase.KG_KS_CHARACTERIZATION;
        subStep = 0;
        cycleCount = 0;
        aborted = false;

        tunedKg = 0;
        tunedKs = 0;
        tunedKv = 0;
        tunedKa = 0;

        consecutiveCurrentViolations = 0;
        angle1Deg = 12.5;
        angle2Deg = 20.0;

        // Move to first test angle
        hood.setPosition(angle1Deg);
    }

    @Override
    public void execute() {
        cycleCount++;

        // Safety checks
        double pos = hood.getCurrentPositionDegrees();
        double vel = hood.getVelocityRPS();
        double current = hood.getStatorCurrent();

        SmartDashboard.putNumber(PREFIX + "Position", pos);
        SmartDashboard.putNumber(PREFIX + "Velocity", vel);
        SmartDashboard.putNumber(PREFIX + "Current", current);

        if (pos < POSITION_MIN_ABORT || pos > POSITION_MAX_ABORT) {
            System.err.println("HOOD AUTO-TUNE: Position out of range (" + fmt(pos) + " deg). Aborting.");
            aborted = true;
            return;
        }
        if (Math.abs(vel) > VELOCITY_ABORT_RPS) {
            System.err.println("HOOD AUTO-TUNE: Velocity too high (" + fmt(vel) + " RPS). Aborting.");
            aborted = true;
            return;
        }
        if (current > CURRENT_ABORT_AMPS) {
            consecutiveCurrentViolations++;
            if (consecutiveCurrentViolations >= CURRENT_ABORT_CONSECUTIVE) {
                System.err.println("HOOD AUTO-TUNE: Sustained overcurrent (" + fmt(current) + "A). Aborting.");
                aborted = true;
                return;
            }
        } else {
            consecutiveCurrentViolations = 0;
        }

        switch (currentPhase) {
        case KG_KS_CHARACTERIZATION:
            executeKgKsCharacterization();
            break;
        case KV_KA_CHARACTERIZATION:
            executeKvKaCharacterization();
            break;
        case VALIDATION:
            executeValidation();
            break;
        case DONE:
            break;
        }
    }

    // ===== PHASE 1: kG + kS via bidirectional ramps =====
    //
    // At a given angle, gravity creates a constant voltage offset V_grav.
    // Friction requires V_friction to overcome in either direction.
    //   Ramp positive until motion: V_up = V_grav + V_friction
    //   Ramp negative until motion: V_down = V_grav - V_friction
    //   V_grav = (V_up + V_down) / 2
    //   V_friction = (V_up - V_down) / 2
    //
    // Do this at two angles to compute kG from the cosine model.

    private void executeKgKsCharacterization() {
        SmartDashboard.putString(PREFIX + "Phase", "kG+kS Characterization");

        switch (subStep) {
        case 0: // Settle at angle 1 (12.5 deg)
            if (cycleCount > SETTLE_CYCLES) {
                hood.setNeutral();
                rampVoltage = 0;
                motionConsecutiveCount = 0;
                subStep = 1;
                cycleCount = 0;
                publishStatus("Phase 1: Ramping positive at " + fmt(angle1Deg) + " deg");
            }
            break;

        case 1: // Ramp positive at angle 1 until motion detected
            rampVoltage += RAMP_RATE;
            if (rampVoltage > RAMP_ABORT_VOLTAGE) {
                System.err.println("HOOD AUTO-TUNE: Positive ramp at angle 1 exceeded limit. Aborting.");
                aborted = true;
                return;
            }
            hood.applyVoltage(rampVoltage);

            if (hood.getVelocityRPS() > MOTION_VELOCITY_THRESHOLD) {
                motionConsecutiveCount++;
            } else {
                motionConsecutiveCount = 0;
            }
            if (motionConsecutiveCount >= MOTION_CONSECUTIVE_SAMPLES) {
                vUp1 = rampVoltage;
                publishStatus("Phase 1: V_up at " + fmt(angle1Deg) + " deg = " + fmt(vUp1) + "V");
                hood.applyVoltage(0);
                // Return to angle 1 with PID
                hood.setPosition(angle1Deg);
                rampVoltage = 0;
                motionConsecutiveCount = 0;
                subStep = 2;
                cycleCount = 0;
            }
            break;

        case 2: // Settle before negative ramp at angle 1
            if (cycleCount > SETTLE_CYCLES) {
                hood.setNeutral();
                subStep = 3;
                cycleCount = 0;
                publishStatus("Phase 1: Ramping negative at " + fmt(angle1Deg) + " deg");
            }
            break;

        case 3: // Ramp negative at angle 1 until motion detected
            rampVoltage += RAMP_RATE;
            if (rampVoltage > RAMP_ABORT_VOLTAGE) {
                System.err.println("HOOD AUTO-TUNE: Negative ramp at angle 1 exceeded limit. Aborting.");
                aborted = true;
                return;
            }
            hood.applyVoltage(-rampVoltage);

            if (hood.getVelocityRPS() < -MOTION_VELOCITY_THRESHOLD) {
                motionConsecutiveCount++;
            } else {
                motionConsecutiveCount = 0;
            }
            if (motionConsecutiveCount >= MOTION_CONSECUTIVE_SAMPLES) {
                vDown1 = -rampVoltage; // store as negative
                publishStatus("Phase 1: V_down at " + fmt(angle1Deg) + " deg = " + fmt(vDown1) + "V");
                hood.applyVoltage(0);
                // Move to angle 2
                hood.setPosition(angle2Deg);
                rampVoltage = 0;
                motionConsecutiveCount = 0;
                subStep = 4;
                cycleCount = 0;
            }
            break;

        case 4: // Settle at angle 2 (20 deg)
            if (cycleCount > SETTLE_CYCLES) {
                hood.setNeutral();
                rampVoltage = 0;
                motionConsecutiveCount = 0;
                subStep = 5;
                cycleCount = 0;
                publishStatus("Phase 1: Ramping positive at " + fmt(angle2Deg) + " deg");
            }
            break;

        case 5: // Ramp positive at angle 2
            rampVoltage += RAMP_RATE;
            if (rampVoltage > RAMP_ABORT_VOLTAGE) {
                System.err.println("HOOD AUTO-TUNE: Positive ramp at angle 2 exceeded limit. Aborting.");
                aborted = true;
                return;
            }
            hood.applyVoltage(rampVoltage);

            if (hood.getVelocityRPS() > MOTION_VELOCITY_THRESHOLD) {
                motionConsecutiveCount++;
            } else {
                motionConsecutiveCount = 0;
            }
            if (motionConsecutiveCount >= MOTION_CONSECUTIVE_SAMPLES) {
                vUp2 = rampVoltage;
                publishStatus("Phase 1: V_up at " + fmt(angle2Deg) + " deg = " + fmt(vUp2) + "V");
                hood.applyVoltage(0);
                hood.setPosition(angle2Deg);
                rampVoltage = 0;
                motionConsecutiveCount = 0;
                subStep = 6;
                cycleCount = 0;
            }
            break;

        case 6: // Settle before negative ramp at angle 2
            if (cycleCount > SETTLE_CYCLES) {
                hood.setNeutral();
                subStep = 7;
                cycleCount = 0;
                publishStatus("Phase 1: Ramping negative at " + fmt(angle2Deg) + " deg");
            }
            break;

        case 7: // Ramp negative at angle 2
            rampVoltage += RAMP_RATE;
            if (rampVoltage > RAMP_ABORT_VOLTAGE) {
                System.err.println("HOOD AUTO-TUNE: Negative ramp at angle 2 exceeded limit. Aborting.");
                aborted = true;
                return;
            }
            hood.applyVoltage(-rampVoltage);

            if (hood.getVelocityRPS() < -MOTION_VELOCITY_THRESHOLD) {
                motionConsecutiveCount++;
            } else {
                motionConsecutiveCount = 0;
            }
            if (motionConsecutiveCount >= MOTION_CONSECUTIVE_SAMPLES) {
                vDown2 = -rampVoltage;
                publishStatus("Phase 1: V_down at " + fmt(angle2Deg) + " deg = " + fmt(vDown2) + "V");
                hood.applyVoltage(0);

                // Compute gravity and friction voltages at each angle
                double vGrav1 = (vUp1 + vDown1) / 2.0;
                double vFric1 = (vUp1 - vDown1) / 2.0;
                double vGrav2 = (vUp2 + vDown2) / 2.0;
                double vFric2 = (vUp2 - vDown2) / 2.0;

                // kS = average friction from both angles
                tunedKs = (vFric1 + vFric2) / 2.0;

                // kG from cosine model: CTRE applies kG * cos(pos_rotations * 2pi)
                double cos1 = Math.cos(angle1Deg / 360.0 * 2.0 * Math.PI);
                double cos2 = Math.cos(angle2Deg / 360.0 * 2.0 * Math.PI);

                // Avoid division by near-zero cosine
                double kg1 = Math.abs(cos1) > 0.01 ? vGrav1 / cos1 : 0;
                double kg2 = Math.abs(cos2) > 0.01 ? vGrav2 / cos2 : 0;

                if (kg1 != 0 && kg2 != 0) {
                    tunedKg = (kg1 + kg2) / 2.0;
                } else if (kg1 != 0) {
                    tunedKg = kg1;
                } else {
                    tunedKg = kg2;
                }

                SmartDashboard.putNumber(PREFIX + "kG", tunedKg);
                SmartDashboard.putNumber(PREFIX + "kS", tunedKs);
                System.out.println("Phase 1 raw data:");
                System.out.println("  Angle " + fmt(angle1Deg) + ": V_up=" + fmt(vUp1) +
                        " V_down=" + fmt(vDown1) + " V_grav=" + fmt(vGrav1) + " V_fric=" + fmt(vFric1));
                System.out.println("  Angle " + fmt(angle2Deg) + ": V_up=" + fmt(vUp2) +
                        " V_down=" + fmt(vDown2) + " V_grav=" + fmt(vGrav2) + " V_fric=" + fmt(vFric2));
                publishStatus("Phase 1: kG=" + fmt(tunedKg) + " kS=" + fmt(tunedKs) + " - verifying");

                // Apply tuned kG+kS and verify at 5 deg
                applyCurrentTuning();
                hood.setPosition(5.0);
                subStep = 8;
                cycleCount = 0;
            }
            break;

        case 8: // Settle at 5 deg with PID+kG, then hold with raw kG voltage
            if (cycleCount < SETTLE_CYCLES) {
                break;
            }
            if (cycleCount == (int) SETTLE_CYCLES) {
                kgVerifyStartPos = hood.getCurrentPositionDegrees();
                // Apply raw gravity compensation voltage (no PID, no friction comp)
                double cosVal = Math.cos(kgVerifyStartPos / 360.0 * 2.0 * Math.PI);
                double verifyVoltage = tunedKg * cosVal;
                hood.applyVoltage(verifyVoltage);
                publishStatus("Phase 1: Verifying - raw " + fmt(verifyVoltage) + "V at " +
                        fmt(kgVerifyStartPos) + " deg");
            }
            if (cycleCount > SETTLE_CYCLES + 50) { // 1s hold
                double drift = Math.abs(hood.getCurrentPositionDegrees() - kgVerifyStartPos);
                if (drift > 3.0) {
                    publishStatus("Phase 1: WARNING - kG verification drifted " + fmt(drift) +
                            " deg (expected for high-ratio gearbox - friction holds position)");
                } else {
                    publishStatus("Phase 1: kG verified (drift=" + fmt(drift) + " deg)");
                }

                hood.applyVoltage(0);
                publishStatus("Phase 1 COMPLETE: kG=" + fmt(tunedKg) + " kS=" + fmt(tunedKs));
                System.out.println("Phase 1 Results: kG=" + fmt(tunedKg) + " kS=" + fmt(tunedKs));

                // Move to kV/kA
                applyCurrentTuning();
                hood.setPosition(2.0);
                currentPhase = Phase.KV_KA_CHARACTERIZATION;
                subStep = 0;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== PHASE 2: kV/kA CHARACTERIZATION =====

    private void executeKvKaCharacterization() {
        SmartDashboard.putString(PREFIX + "Phase", "kV/kA Characterization");

        switch (subStep) {
        case 0: // Settle at 2 deg, prepare test voltages
            if (cycleCount > SETTLE_CYCLES) {
                // Test voltages above what's needed to overcome gravity + friction
                double baseVoltage = Math.abs(tunedKg) + tunedKs;
                kvTestVoltages = new double[] {
                        baseVoltage + 0.2,
                        baseVoltage + 0.4,
                        baseVoltage + 0.6
                };
                kvMeasuredVelocities = new double[3];
                kvTestIndex = 0;
                kvVelocityAccum = 0;
                kvMeasureCount = 0;
                subStep = 1;
                cycleCount = 0;
                publishStatus("Phase 2: Testing kV with 3 voltage levels");
            }
            break;

        case 1: { // Apply test voltage, settle 0.5s, measure 0.2s, or stop if pos > 22
            double testV = Math.min(kvTestVoltages[kvTestIndex], MAX_CHARACTERIZATION_VOLTAGE);
            hood.applyVoltage(testV);
            double elapsed = cycleCount / 50.0;
            double pos = hood.getCurrentPositionDegrees();

            if (elapsed < 0.5) {
                // Settling - motor accelerating
            } else if (elapsed < 0.7 && pos < 22.0) {
                kvVelocityAccum += Math.abs(hood.getVelocityRPS());
                kvMeasureCount++;
            }
            if (elapsed >= 0.7 || pos >= 22.0) {
                double avgVel = kvMeasureCount > 0 ? kvVelocityAccum / kvMeasureCount : 0;
                kvMeasuredVelocities[kvTestIndex] = avgVel;
                hood.applyVoltage(0);
                kvTestIndex++;
                kvVelocityAccum = 0;
                kvMeasureCount = 0;

                if (kvTestIndex < 3) {
                    // Return to 2 deg for next test
                    applyCurrentTuning();
                    hood.setPosition(2.0);
                    subStep = 10; // settle substep
                    cycleCount = 0;
                } else {
                    // Compute kV: net_voltage / velocity
                    double kvSum = 0;
                    int kvValid = 0;
                    for (int i = 0; i < 3; i++) {
                        if (kvMeasuredVelocities[i] > 0.005) {
                            double appliedV = Math.min(kvTestVoltages[i], MAX_CHARACTERIZATION_VOLTAGE);
                            double netVoltage = appliedV - Math.abs(tunedKg) - tunedKs;
                            if (netVoltage > 0) {
                                kvSum += netVoltage / kvMeasuredVelocities[i];
                                kvValid++;
                            }
                        }
                    }
                    tunedKv = kvValid > 0 ? kvSum / kvValid : 0;
                    tunedKv = Math.max(0, Math.min(50.0, tunedKv));
                    SmartDashboard.putNumber(PREFIX + "kV", tunedKv);
                    publishStatus("Phase 2: kV = " + fmt(tunedKv) + " - measuring kA");

                    // Return to 2 deg for kA test
                    applyCurrentTuning();
                    hood.setPosition(2.0);
                    subStep = 2;
                    cycleCount = 0;
                }
            }
            break;
        }

        case 10: // Settle between kV tests
            if (cycleCount > SETTLE_CYCLES) {
                subStep = 1;
                cycleCount = 0;
            }
            break;

        case 2: // Settle before kA test
            if (cycleCount > SETTLE_CYCLES) {
                subStep = 3;
                cycleCount = 0;
                kaPrevVelocity = 0;
                kaAccelSum = 0;
                kaAccelCount = 0;
            }
            break;

        case 3: { // kA: step voltage, measure acceleration over 0.15s
            double stepVoltage = Math.min(Math.abs(tunedKg) + tunedKs + 0.5, MAX_CHARACTERIZATION_VOLTAGE);
            hood.applyVoltage(stepVoltage);
            double elapsed = cycleCount / 50.0;

            if (elapsed > 0.02 && elapsed < 0.17) {
                double vel = Math.abs(hood.getVelocityRPS());
                if (kaPrevVelocity > 0) {
                    double accel = (vel - kaPrevVelocity) * 50.0; // dv/dt
                    if (accel > 0) {
                        kaAccelSum += accel;
                        kaAccelCount++;
                    }
                }
                kaPrevVelocity = vel;
            } else if (elapsed >= 0.17) {
                hood.applyVoltage(0);
                if (kaAccelCount > 0) {
                    double avgAccel = kaAccelSum / kaAccelCount;
                    double netVoltage = stepVoltage - Math.abs(tunedKg) - tunedKs;
                    if (avgAccel > 0.01 && netVoltage > 0) {
                        tunedKa = netVoltage / avgAccel;
                    }
                }
                tunedKa = Math.max(0, Math.min(5.0, tunedKa));
                SmartDashboard.putNumber(PREFIX + "kA", tunedKa);

                publishStatus("Phase 2 COMPLETE: kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));
                System.out.println("Phase 2 Results: kV=" + fmt(tunedKv) + " kA=" + fmt(tunedKa));

                // Apply all FF and move to validation
                applyCurrentTuning();
                hood.setPosition(0);
                currentPhase = Phase.VALIDATION;
                subStep = 0;
                cycleCount = 0;
                initValidation();
            }
            break;
        }
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== PHASE 3: VALIDATION =====

    private void initValidation() {
        validationTargets = new double[] { 12.5, 25.0, 5.0, 20.0, 0.0 };
        validationIndex = 0;
        publishStatus("Phase 3: Validation - " + validationTargets.length + " step tests");
    }

    private void executeValidation() {
        SmartDashboard.putString(PREFIX + "Phase", "Validation");

        switch (subStep) {
        case 0: // Settle before next step test
            if (cycleCount > SETTLE_CYCLES) {
                if (validationIndex < validationTargets.length) {
                    double target = validationTargets[validationIndex];
                    double currentPos = hood.getCurrentPositionDegrees();
                    double stepSizeRot = Math.abs(target - currentPos) / 360.0;
                    double tolerance = Math.max(0.5 / 360.0, stepSizeRot * 0.05);
                    analyzer = new StepResponseAnalyzer(currentPos / 360.0, target / 360.0, tolerance);
                    stepStartTime = System.nanoTime() / 1e9;
                    hood.setPosition(target);
                    subStep = 1;
                    cycleCount = 0;
                    publishStatus("Phase 3: Step " + (validationIndex + 1) + "/" +
                            validationTargets.length + " -> " + fmt(target) + " deg");
                } else {
                    publishResults();
                    currentPhase = Phase.DONE;
                }
            }
            break;

        case 1: // Record step response
            double now = System.nanoTime() / 1e9;
            double posRot = hood.getCurrentPositionDegrees() / 360.0;
            analyzer.addSample(posRot);

            if (now - stepStartTime >= STEP_RECORD_SECONDS) {
                double overshoot = analyzer.getOvershootPercent();
                double settlingTime = analyzer.getSettlingTime();
                double riseTime = analyzer.getRiseTime();
                double ssError = analyzer.getSteadyStateError();
                publishStatus("Phase 3: Step " + (validationIndex + 1) +
                        " overshoot=" + String.format("%.1f", overshoot) + "%" +
                        " settling=" + fmt(settlingTime) + "s" +
                        " rise=" + fmt(riseTime) + "s" +
                        " error=" + fmt(ssError * 360.0) + " deg");
                SmartDashboard.putNumber(PREFIX + "Validation/Overshoot", overshoot);
                SmartDashboard.putNumber(PREFIX + "Validation/SettlingTime", settlingTime);
                SmartDashboard.putNumber(PREFIX + "Validation/RiseTime", riseTime);
                SmartDashboard.putNumber(PREFIX + "Validation/SteadyStateError", ssError * 360.0);
                validationIndex++;
                subStep = 0;
                cycleCount = 0;
            }
            break;
        }

        SmartDashboard.putNumber(PREFIX + "Progress", getProgress());
    }

    // ===== RESULTS =====

    private void publishResults() {
        System.out.println("\n====================================");
        System.out.println("   HOOD FF AUTO-TUNE RESULTS");
        System.out.println("====================================");
        System.out.println("// Paste into Constants.java HoodConstants:");
        System.out.println("public static final double S = " + fmt(tunedKs) + ";");
        System.out.println("public static final double V = " + fmt(tunedKv) + ";");
        System.out.println("public static final double A = " + fmt(tunedKa) + ";");
        System.out.println("public static final double G = " + fmt(tunedKg) + ";");
        System.out.println("====================================\n");

        SmartDashboard.putNumber(PREFIX + "Results/kS", tunedKs);
        SmartDashboard.putNumber(PREFIX + "Results/kV", tunedKv);
        SmartDashboard.putNumber(PREFIX + "Results/kA", tunedKa);
        SmartDashboard.putNumber(PREFIX + "Results/kG", tunedKg);

        publishStatus("COMPLETE - Tuned FF gains are active. See console for copy-paste values.");
        SmartDashboard.putNumber(PREFIX + "Progress", 1.0);
        System.out.println("=== HOOD FF AUTO-TUNE COMPLETE - Gains are active for testing ===");
    }

    // ===== LIFECYCLE =====

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE || aborted;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || aborted) {
            hood.restoreDefaultConfig();
            hood.setNeutral();
            publishStatus("CANCELLED - Original gains restored");
            System.out.println("=== HOOD FF AUTO-TUNE CANCELLED - Original gains restored ===");
        } else {
            // Leave tuned gains active so operator can test
            hood.setNeutral();
        }
    }

    // ===== UTILITIES =====

    /** Apply all currently-tuned FF gains with existing PID constants. */
    private void applyCurrentTuning() {
        hood.applyTuningConfig(
                tunedKs, tunedKv, tunedKa, tunedKg,
                HoodConstants.P, HoodConstants.I, HoodConstants.D,
                HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY,
                HoodConstants.MOTION_MAGIC_ACCELERATION,
                HoodConstants.MOTION_MAGIC_JERK);
    }

    private double getProgress() {
        switch (currentPhase) {
        case KG_KS_CHARACTERIZATION:
            return subStep / 36.0; // ~0 to 0.25
        case KV_KA_CHARACTERIZATION:
            return 0.25 + (subStep / 16.0); // ~0.25 to 0.50
        case VALIDATION:
            return 0.50 + ((double) validationIndex / validationTargets.length) * 0.50;
        case DONE:
            return 1.0;
        default:
            return 0;
        }
    }

    private void publishStatus(String status) {
        SmartDashboard.putString(PREFIX + "Status", status);
        System.out.println("[HoodAutoTune] " + status);
    }

    private static String fmt(double v) {
        return String.format("%.4f", v);
    }
}
