package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.util.DiagnosticLogger;

/**
 * Flywheel system identification command for MATLAB. Runs a multi-phase test sequence
 * collecting quasistatic, dynamic step, coast-down, and current-domain data. All data
 * is logged to CSV via DiagnosticLogger for import into MATLAB's System Identification
 * Toolbox and PID Tuner.
 *
 * <p>Bind to a button in test mode. Press again to cancel (partial CSV is still saved).
 * Total runtime ~2-2.5 minutes.
 */
public class FlywheelSysIdCommand extends Command {

    // Safety limits
    private static final double MAX_VOLTAGE = 10.0;
    private static final double MAX_VELOCITY_RPS = 100.0; // ~6000 RPM
    private static final double STATOR_CURRENT_ABORT = 100.0;
    private static final int MAX_STATOR_VIOLATIONS = 20;

    // Quasistatic ramp parameters
    private static final double QS_RAMP_RATE_V_PER_SEC = 0.5; // 0→10V in 20s
    private static final double QS_MAX_VOLTAGE = 10.0;

    // Voltage step test parameters
    private static final double[] VOLTAGE_STEPS = { 2.0, 4.0, 6.0, 8.0, 10.0 };
    private static final double VOLTAGE_STEP_HOLD_SECONDS = 4.0;
    private static final double VOLTAGE_STEP_SETTLE_SECONDS = 1.0;

    // Coast-down parameters
    private static final double COASTDOWN_SPINUP_VOLTAGE = 10.0;
    private static final double COASTDOWN_SPINUP_SECONDS = 3.0;
    private static final double COASTDOWN_MIN_VELOCITY_RPS = 0.5;
    private static final double COASTDOWN_TIMEOUT_SECONDS = 30.0;

    // Current step test parameters
    private static final double[] CURRENT_STEPS = { 5.0, 10.0, 15.0, 20.0, 30.0 };
    private static final double CURRENT_STEP_HOLD_SECONDS = 4.0;
    private static final double CURRENT_STEP_SETTLE_SECONDS = 1.0;

    // Settle parameters
    private static final double SETTLE_VELOCITY_THRESHOLD_RPS = 0.5;
    private static final double SETTLE_TIMEOUT_SECONDS = 15.0;

    // Phase IDs for CSV
    private static final double PHASE_ID_QUASISTATIC = 0.0;
    private static final double PHASE_ID_VOLTAGE_STEPS = 1.0;
    private static final double PHASE_ID_COASTDOWN = 2.0;
    private static final double PHASE_ID_CURRENT_STEPS = 3.0;

    // Command type IDs for CSV
    private static final double CMD_VOLTAGE = 0.0;
    private static final double CMD_CURRENT = 1.0;
    private static final double CMD_NONE = 2.0;

    private static final String PREFIX = "FlywheelSysId/";

    private final Flywheel flywheel;
    private final DiagnosticLogger logger;

    private enum Phase {
        QUASISTATIC_FORWARD,
        SETTLE_AFTER_QS,
        VOLTAGE_STEPS_HOLD,
        VOLTAGE_STEPS_SETTLE,
        SETTLE_AFTER_VSTEPS,
        COASTDOWN_SPINUP,
        COASTDOWN,
        SETTLE_AFTER_COAST,
        CURRENT_STEPS_HOLD,
        CURRENT_STEPS_SETTLE,
        SETTLE_AFTER_CSTEPS,
        DONE
    }

    private Phase currentPhase;
    private int stepIndex;
    private double phaseStartTime;
    private int consecutiveStatorViolations;

    public FlywheelSysIdCommand(Flywheel flywheel) {
        this.flywheel = flywheel;
        this.logger = new DiagnosticLogger("flywheel_sysid", new String[] {
                "timestamp",
                "test_phase",
                "command_type",
                "command_value",
                "velocity_rps",
                "velocity_rpm",
                "motor_voltage",
                "stator_current",
                "supply_voltage"
        });
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        System.out.println("=== FLYWHEEL SYSID STARTING ===");
        logger.open();
        currentPhase = Phase.QUASISTATIC_FORWARD;
        stepIndex = 0;
        phaseStartTime = Timer.getFPGATimestamp();
        consecutiveStatorViolations = 0;
        flywheel.stop();
        publishStatus("Starting quasistatic ramp");
    }

    @Override
    public void execute() {
        // Safety checks
        if (!safetyCheck()) {
            return;
        }

        switch (currentPhase) {
            case QUASISTATIC_FORWARD:
                executeQuasistaticForward();
                break;
            case SETTLE_AFTER_QS:
                executeSettle(Phase.VOLTAGE_STEPS_HOLD, "Voltage steps");
                break;
            case VOLTAGE_STEPS_HOLD:
                executeVoltageStepHold();
                break;
            case VOLTAGE_STEPS_SETTLE:
                executeVoltageStepSettle();
                break;
            case SETTLE_AFTER_VSTEPS:
                executeSettle(Phase.COASTDOWN_SPINUP, "Coast-down spinup");
                break;
            case COASTDOWN_SPINUP:
                executeCoastdownSpinup();
                break;
            case COASTDOWN:
                executeCoastdown();
                break;
            case SETTLE_AFTER_COAST:
                executeSettle(Phase.CURRENT_STEPS_HOLD, "Current steps");
                break;
            case CURRENT_STEPS_HOLD:
                executeCurrentStepHold();
                break;
            case CURRENT_STEPS_SETTLE:
                executeCurrentStepSettle();
                break;
            case SETTLE_AFTER_CSTEPS:
                executeSettle(Phase.DONE, "Done");
                break;
            case DONE:
                break;
        }
    }

    // --- Phase implementations ---

    private void executeQuasistaticForward() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = Math.min(elapsed * QS_RAMP_RATE_V_PER_SEC, QS_MAX_VOLTAGE);

        flywheel.applyVoltage(voltage);
        logDataPoint(PHASE_ID_QUASISTATIC, CMD_VOLTAGE, voltage);

        if (voltage >= QS_MAX_VOLTAGE) {
            transitionTo(Phase.SETTLE_AFTER_QS, "Settling after quasistatic");
            flywheel.stop();
        }
    }

    private void executeVoltageStepHold() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = VOLTAGE_STEPS[stepIndex];

        flywheel.applyVoltage(voltage);
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_VOLTAGE, voltage);

        if (elapsed >= VOLTAGE_STEP_HOLD_SECONDS) {
            flywheel.stop();
            transitionTo(Phase.VOLTAGE_STEPS_SETTLE, "Voltage step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length + " settle");
        }
    }

    private void executeVoltageStepSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_NONE, 0.0);

        if (elapsed >= VOLTAGE_STEP_SETTLE_SECONDS) {
            stepIndex++;
            if (stepIndex >= VOLTAGE_STEPS.length) {
                stepIndex = 0;
                transitionTo(Phase.SETTLE_AFTER_VSTEPS, "Settling after voltage steps");
                flywheel.stop();
            } else {
                transitionTo(Phase.VOLTAGE_STEPS_HOLD, "Voltage step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length);
            }
        }
    }

    private void executeCoastdownSpinup() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;

        flywheel.applyVoltage(COASTDOWN_SPINUP_VOLTAGE);
        logDataPoint(PHASE_ID_COASTDOWN, CMD_VOLTAGE, COASTDOWN_SPINUP_VOLTAGE);

        if (elapsed >= COASTDOWN_SPINUP_SECONDS) {
            flywheel.stop();
            transitionTo(Phase.COASTDOWN, "Coast-down recording");
        }
    }

    private void executeCoastdown() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(flywheel.getVelocityRPS());

        logDataPoint(PHASE_ID_COASTDOWN, CMD_NONE, 0.0);

        if (velocityRPS < COASTDOWN_MIN_VELOCITY_RPS || elapsed >= COASTDOWN_TIMEOUT_SECONDS) {
            transitionTo(Phase.SETTLE_AFTER_COAST, "Settling after coast-down");
            flywheel.stop();
        }
    }

    private void executeCurrentStepHold() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double current = CURRENT_STEPS[stepIndex];

        flywheel.applyCurrent(current);
        logDataPoint(PHASE_ID_CURRENT_STEPS, CMD_CURRENT, current);

        if (elapsed >= CURRENT_STEP_HOLD_SECONDS) {
            flywheel.stop();
            transitionTo(Phase.CURRENT_STEPS_SETTLE, "Current step " + (stepIndex + 1) + "/" + CURRENT_STEPS.length + " settle");
        }
    }

    private void executeCurrentStepSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        logDataPoint(PHASE_ID_CURRENT_STEPS, CMD_NONE, 0.0);

        if (elapsed >= CURRENT_STEP_SETTLE_SECONDS) {
            stepIndex++;
            if (stepIndex >= CURRENT_STEPS.length) {
                stepIndex = 0;
                transitionTo(Phase.SETTLE_AFTER_CSTEPS, "Settling after current steps");
                flywheel.stop();
            } else {
                transitionTo(Phase.CURRENT_STEPS_HOLD, "Current step " + (stepIndex + 1) + "/" + CURRENT_STEPS.length);
            }
        }
    }

    private void executeSettle(Phase nextPhase, String nextDescription) {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(flywheel.getVelocityRPS());

        // Keep logging during settle for complete data
        double currentPhaseId;
        if (currentPhase == Phase.SETTLE_AFTER_QS) {
            currentPhaseId = PHASE_ID_QUASISTATIC;
        } else if (currentPhase == Phase.SETTLE_AFTER_VSTEPS) {
            currentPhaseId = PHASE_ID_VOLTAGE_STEPS;
        } else if (currentPhase == Phase.SETTLE_AFTER_COAST) {
            currentPhaseId = PHASE_ID_COASTDOWN;
        } else {
            currentPhaseId = PHASE_ID_CURRENT_STEPS;
        }
        logDataPoint(currentPhaseId, CMD_NONE, 0.0);

        if (velocityRPS < SETTLE_VELOCITY_THRESHOLD_RPS || elapsed >= SETTLE_TIMEOUT_SECONDS) {
            transitionTo(nextPhase, nextDescription);
        }
    }

    // --- Helpers ---

    private boolean safetyCheck() {
        double velocityRPS = Math.abs(flywheel.getVelocityRPS());
        double statorCurrent = Math.abs(flywheel.getStatorCurrent());

        if (velocityRPS > MAX_VELOCITY_RPS) {
            System.err.println("FLYWHEEL SYSID: Velocity exceeded " + MAX_VELOCITY_RPS + " RPS. Aborting.");
            cancel();
            return false;
        }

        if (statorCurrent > STATOR_CURRENT_ABORT) {
            consecutiveStatorViolations++;
            if (consecutiveStatorViolations >= MAX_STATOR_VIOLATIONS) {
                System.err.println("FLYWHEEL SYSID: Stator current exceeded " + STATOR_CURRENT_ABORT + "A. Aborting.");
                cancel();
                return false;
            }
        } else {
            consecutiveStatorViolations = 0;
        }

        return true;
    }

    private void logDataPoint(double phaseId, double commandType, double commandValue) {
        double timestamp = Timer.getFPGATimestamp();
        double velocityRPS = flywheel.getVelocityRPS();
        double velocityRPM = velocityRPS * 60.0;
        double motorVoltage = flywheel.getMotorVoltage();
        double statorCurrent = flywheel.getStatorCurrent();
        double supplyVoltage = flywheel.getSupplyVoltage();

        logger.logRow(
                timestamp,
                phaseId,
                commandType,
                commandValue,
                velocityRPS,
                velocityRPM,
                motorVoltage,
                statorCurrent,
                supplyVoltage);
    }

    private void transitionTo(Phase phase, String description) {
        currentPhase = phase;
        phaseStartTime = Timer.getFPGATimestamp();
        publishStatus(description);
        System.out.println("FlywheelSysId: " + description);
    }

    private void publishStatus(String status) {
        SmartDashboard.putString(PREFIX + "Status", status);
        SmartDashboard.putString(PREFIX + "Phase", currentPhase.name());
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        logger.close();
        if (interrupted) {
            publishStatus("CANCELLED - partial data saved");
            System.out.println("=== FLYWHEEL SYSID CANCELLED ===");
        } else {
            publishStatus("COMPLETE - data saved");
            System.out.println("=== FLYWHEEL SYSID COMPLETE ===");
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }
}
