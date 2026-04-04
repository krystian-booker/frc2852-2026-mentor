package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.DiagnosticLogger;

/**
 * Turret system identification command for MATLAB. Collects voltage-response data
 * in three independent routines (quasistatic ramp, voltage steps, coast-down) that
 * can be run separately. Data is logged to CSV for import into turret_sysid_analysis.m.
 *
 * <p>The turret has limited rotation range (~370 degrees), so this command includes
 * a position guard that cleanly ends phases before hitting soft limits. Voltage steps
 * are bidirectional (+V then -V) to keep the turret near its starting position.
 */
public class TurretSysIdCommand extends Command {

    public enum Routine {
        QUASISTATIC,
        STEPS,
        COASTDOWN
    }

    // Safety limits
    private static final double MAX_VELOCITY_RPS = 3.0; // Turret max is ~2 RPS at 50:1
    private static final double STATOR_CURRENT_ABORT = 80.0;
    private static final int MAX_STATOR_VIOLATIONS = 20;
    private static final double POSITION_LIMIT_DEGREES = 170.0; // Buffer before ±185 soft limit

    // Quasistatic ramp parameters
    private static final double QS_RAMP_RATE_V_PER_SEC = 0.3; // Slower for more data before position limit
    private static final double QS_MAX_VOLTAGE = 3.0;

    // Voltage step test parameters (bidirectional: +V then -V per level)
    private static final double[] VOLTAGE_STEPS = { 1.5, 2.5, 3.5 };
    private static final double VOLTAGE_STEP_HOLD_SECONDS = 1.5;
    private static final double VOLTAGE_STEP_SETTLE_SECONDS = 1.0;

    // Coast-down parameters
    private static final double COASTDOWN_SPINUP_VOLTAGE = 4.0;
    private static final double COASTDOWN_SPINUP_SECONDS = 1.5;
    private static final double COASTDOWN_MIN_VELOCITY_RPS = 0.05;
    private static final double COASTDOWN_TIMEOUT_SECONDS = 5.0;

    // Settle parameters
    private static final double SETTLE_VELOCITY_THRESHOLD_RPS = 0.05;
    private static final double SETTLE_TIMEOUT_SECONDS = 5.0;

    // Phase IDs for CSV
    private static final double PHASE_ID_QUASISTATIC = 0.0;
    private static final double PHASE_ID_VOLTAGE_STEPS = 1.0;
    private static final double PHASE_ID_COASTDOWN = 2.0;

    // Command type IDs for CSV
    private static final double CMD_VOLTAGE = 0.0;
    private static final double CMD_NONE = 2.0;

    private static final String PREFIX = "TurretSysId/";

    private final Turret turret;
    private final DiagnosticLogger logger;
    private final Routine routineToRun;

    private enum Phase {
        QUASISTATIC_FORWARD,
        VOLTAGE_STEPS_POS_HOLD,
        VOLTAGE_STEPS_POS_SETTLE,
        VOLTAGE_STEPS_NEG_HOLD,
        VOLTAGE_STEPS_NEG_SETTLE,
        COASTDOWN_SPINUP,
        COASTDOWN,
        SETTLING,
        DONE
    }

    private Phase currentPhase;
    private int stepIndex;
    private double phaseStartTime;
    private int consecutiveStatorViolations;

    public TurretSysIdCommand(Turret turret, Routine routine) {
        this.turret = turret;
        this.routineToRun = routine;

        String logPrefix = "turret_sysid_";
        switch (routine) {
            case QUASISTATIC: logPrefix += "quasistatic"; break;
            case STEPS:       logPrefix += "steps"; break;
            case COASTDOWN:   logPrefix += "coast"; break;
        }

        this.logger = new DiagnosticLogger(logPrefix, new String[] {
                "timestamp",
                "test_phase",
                "command_type",
                "command_value",
                "turret_velocity_rps",
                "turret_position_rot",
                "motor_voltage",
                "stator_current",
                "supply_voltage"
        });
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        System.out.println("=== TURRET SYSID STARTING (" + routineToRun.name() + ") ===");
        System.out.println("  Encoder position: " + String.format("%.1f", turret.getEncoderDegrees()) + " deg");
        System.out.println("  Position limit guard: +/-" + POSITION_LIMIT_DEGREES + " deg");
        logger.open();

        switch (routineToRun) {
            case QUASISTATIC:
                currentPhase = Phase.QUASISTATIC_FORWARD;
                break;
            case STEPS:
                currentPhase = Phase.VOLTAGE_STEPS_POS_HOLD;
                break;
            case COASTDOWN:
                currentPhase = Phase.COASTDOWN_SPINUP;
                break;
        }

        stepIndex = 0;
        phaseStartTime = Timer.getFPGATimestamp();
        consecutiveStatorViolations = 0;
        turret.stop();
        publishStatus("Starting routine " + routineToRun.name());
    }

    @Override
    public void execute() {
        if (!safetyCheck()) return;

        switch (currentPhase) {
            case QUASISTATIC_FORWARD:
                executeQuasistaticForward();
                break;
            case VOLTAGE_STEPS_POS_HOLD:
                executeVoltageStepPosHold();
                break;
            case VOLTAGE_STEPS_POS_SETTLE:
                executeVoltageStepPosSettle();
                break;
            case VOLTAGE_STEPS_NEG_HOLD:
                executeVoltageStepNegHold();
                break;
            case VOLTAGE_STEPS_NEG_SETTLE:
                executeVoltageStepNegSettle();
                break;
            case COASTDOWN_SPINUP:
                executeCoastdownSpinup();
                break;
            case COASTDOWN:
                executeCoastdown();
                break;
            case SETTLING:
                executeSettle();
                break;
            case DONE:
                break;
        }
    }

    // --- Phase implementations ---

    private void executeQuasistaticForward() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = Math.min(elapsed * QS_RAMP_RATE_V_PER_SEC, QS_MAX_VOLTAGE);

        if (!positionInRange()) {
            transitionTo(Phase.SETTLING, "Position limit reached during quasistatic");
            turret.stop();
            return;
        }

        applyVoltage(voltage);
        logDataPoint(PHASE_ID_QUASISTATIC, CMD_VOLTAGE, voltage);

        if (voltage >= QS_MAX_VOLTAGE) {
            transitionTo(Phase.SETTLING, "Settling after quasistatic");
            turret.stop();
        }
    }

    private void executeVoltageStepPosHold() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = VOLTAGE_STEPS[stepIndex];

        if (!positionInRange()) {
            transitionTo(Phase.VOLTAGE_STEPS_POS_SETTLE, "Position limit - ending positive step early");
            turret.stop();
            return;
        }

        applyVoltage(voltage);
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_VOLTAGE, voltage);

        if (elapsed >= VOLTAGE_STEP_HOLD_SECONDS) {
            turret.stop();
            transitionTo(Phase.VOLTAGE_STEPS_POS_SETTLE,
                    "Step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length + " +V settle");
        }
    }

    private void executeVoltageStepPosSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_NONE, 0.0);

        if (elapsed >= VOLTAGE_STEP_SETTLE_SECONDS) {
            transitionTo(Phase.VOLTAGE_STEPS_NEG_HOLD,
                    "Step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length + " -V");
        }
    }

    private void executeVoltageStepNegHold() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = -VOLTAGE_STEPS[stepIndex];

        if (!positionInRange()) {
            transitionTo(Phase.VOLTAGE_STEPS_NEG_SETTLE, "Position limit - ending negative step early");
            turret.stop();
            return;
        }

        applyVoltage(voltage);
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_VOLTAGE, voltage);

        if (elapsed >= VOLTAGE_STEP_HOLD_SECONDS) {
            turret.stop();
            transitionTo(Phase.VOLTAGE_STEPS_NEG_SETTLE,
                    "Step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length + " -V settle");
        }
    }

    private void executeVoltageStepNegSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_NONE, 0.0);

        if (elapsed >= VOLTAGE_STEP_SETTLE_SECONDS) {
            stepIndex++;
            if (stepIndex >= VOLTAGE_STEPS.length) {
                stepIndex = 0;
                transitionTo(Phase.SETTLING, "Settling after voltage steps");
                turret.stop();
            } else {
                transitionTo(Phase.VOLTAGE_STEPS_POS_HOLD,
                        "Step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length + " +V");
            }
        }
    }

    private void executeCoastdownSpinup() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;

        if (!positionInRange()) {
            // Hit limit during spinup — go straight to coast
            applyVoltage(0.0); // VoltageOut(0) = coast, not brake
            transitionTo(Phase.COASTDOWN, "Position limit during spinup - coasting");
            return;
        }

        applyVoltage(COASTDOWN_SPINUP_VOLTAGE);
        logDataPoint(PHASE_ID_COASTDOWN, CMD_VOLTAGE, COASTDOWN_SPINUP_VOLTAGE);

        if (elapsed >= COASTDOWN_SPINUP_SECONDS) {
            applyVoltage(0.0); // Coast (VoltageOut 0V), not brake
            transitionTo(Phase.COASTDOWN, "Coast-down recording");
        }
    }

    private void executeCoastdown() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(turret.getVelocityRPS());

        logDataPoint(PHASE_ID_COASTDOWN, CMD_NONE, 0.0);

        if (velocityRPS < COASTDOWN_MIN_VELOCITY_RPS || elapsed >= COASTDOWN_TIMEOUT_SECONDS) {
            transitionTo(Phase.SETTLING, "Settling after coast-down");
            turret.stop();
        }
    }

    private void executeSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(turret.getVelocityRPS());

        double currentPhaseId = 0;
        if (routineToRun == Routine.QUASISTATIC) currentPhaseId = PHASE_ID_QUASISTATIC;
        else if (routineToRun == Routine.STEPS) currentPhaseId = PHASE_ID_VOLTAGE_STEPS;
        else if (routineToRun == Routine.COASTDOWN) currentPhaseId = PHASE_ID_COASTDOWN;

        logDataPoint(currentPhaseId, CMD_NONE, 0.0);

        if (velocityRPS < SETTLE_VELOCITY_THRESHOLD_RPS || elapsed >= SETTLE_TIMEOUT_SECONDS) {
            transitionTo(Phase.DONE, "Done");
        }
    }

    // --- Helpers ---

    private void applyVoltage(double voltage) {
        turret.applyVoltage(voltage);
    }

    private boolean positionInRange() {
        double encoderDeg = turret.getEncoderDegrees();
        return Math.abs(encoderDeg) < POSITION_LIMIT_DEGREES;
    }

    private boolean safetyCheck() {
        double velocityRPS = Math.abs(turret.getVelocityRPS());
        double statorCurrent = Math.abs(turret.getStatorCurrent());

        if (velocityRPS > MAX_VELOCITY_RPS) {
            System.err.println("TURRET SYSID: Velocity exceeded " + MAX_VELOCITY_RPS + " rps. Aborting.");
            cancel();
            return false;
        }

        if (statorCurrent > STATOR_CURRENT_ABORT) {
            consecutiveStatorViolations++;
            if (consecutiveStatorViolations >= MAX_STATOR_VIOLATIONS) {
                System.err.println("TURRET SYSID: Stator current exceeded " + STATOR_CURRENT_ABORT + "A. Aborting.");
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
        double velocityRPS = turret.getVelocityRPS();
        double positionRot = turret.getEncoderDegrees() / 360.0;
        double motorVoltage = turret.getMotorVoltage();
        double statorCurrent = turret.getStatorCurrent();
        double supplyVoltage = RobotController.getBatteryVoltage();

        logger.logRow(
                timestamp,
                phaseId,
                commandType,
                commandValue,
                velocityRPS,
                positionRot,
                motorVoltage,
                statorCurrent,
                supplyVoltage);
    }

    private void transitionTo(Phase phase, String description) {
        currentPhase = phase;
        phaseStartTime = Timer.getFPGATimestamp();
        publishStatus(description);
        System.out.println("TurretSysId: " + description);
    }

    private void publishStatus(String status) {
        SmartDashboard.putString(PREFIX + "Status", status);
        SmartDashboard.putString(PREFIX + "Routine", routineToRun.name());
        SmartDashboard.putString(PREFIX + "Phase", currentPhase.name());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        logger.close();
        if (interrupted) {
            publishStatus("CANCELLED - partial data saved");
            System.out.println("=== TURRET SYSID CANCELLED ===");
        } else {
            publishStatus("COMPLETE - data saved");
            System.out.println("=== TURRET SYSID COMPLETE ===");
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }
}
