package frc.robot.commands.tuning;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DiagnosticLogger;

/**
 * Steer system identification command for MATLAB. Refactored into isolated test routines
 * so that each can be executed manually. This exclusively spins the steer (azimuth)
 * wheels to profile their position/velocity mechanics for PID tuning.
 * All data is logged to CSV via DiagnosticLogger for import into MATLAB.
 */
public class SteerSysIdCommand extends Command {

    public enum Routine {
        QUASISTATIC,
        STEPS,
        COASTDOWN
    }

    // Safety limits for steer motor
    private static final double MAX_VELOCITY_RPS = 100.0; // Rotations per sec
    private static final double STATOR_CURRENT_ABORT = 80.0;
    private static final int MAX_STATOR_VIOLATIONS = 20;

    // Quasistatic ramp parameters
    private static final double QS_RAMP_RATE_V_PER_SEC = 0.5; // 0→4V in 8s
    private static final double QS_MAX_VOLTAGE = 4.0;

    // Voltage step test parameters
    private static final double[] VOLTAGE_STEPS = { 2.0, 4.0, 6.0 };
    private static final double VOLTAGE_STEP_HOLD_SECONDS = 1.5;
    private static final double VOLTAGE_STEP_SETTLE_SECONDS = 1.0;

    // Coast-down parameters
    private static final double COASTDOWN_SPINUP_VOLTAGE = 6.0;
    private static final double COASTDOWN_SPINUP_SECONDS = 1.5;
    private static final double COASTDOWN_MIN_VELOCITY_RPS = 0.1;
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

    private static final String PREFIX = "SteerSysId/";

    private final CommandSwerveDrivetrain drivetrain;
    private final DiagnosticLogger logger;
    private final Routine routineToRun;

    // Characterization request specific to Phoenix 6 Steer
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private enum Phase {
        QUASISTATIC_FORWARD,
        VOLTAGE_STEPS_HOLD,
        VOLTAGE_STEPS_SETTLE,
        COASTDOWN_SPINUP,
        COASTDOWN,
        SETTLING,
        DONE
    }

    private Phase currentPhase;
    private int stepIndex;
    private double phaseStartTime;
    private int consecutiveStatorViolations;

    public SteerSysIdCommand(CommandSwerveDrivetrain drivetrain, Routine routine) {
        this.drivetrain = drivetrain;
        this.routineToRun = routine;
        
        String logPrefix = "steer_sysid_";
        switch(routine) {
            case QUASISTATIC: logPrefix += "quasistatic"; break;
            case STEPS: logPrefix += "steps"; break;
            case COASTDOWN: logPrefix += "coast"; break;
        }

        this.logger = new DiagnosticLogger(logPrefix, new String[] {
                "timestamp",
                "test_phase",
                "command_type",
                "command_value",
                "steer_velocity_rps",
                "steer_position_rot",
                "steer_voltage",
                "steer_stator_current",
                "supply_voltage"
        });
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("=== STEER SYSID STARTING (" + routineToRun.name() + ") ===");
        logger.open();
        
        switch(routineToRun) {
            case QUASISTATIC:
                currentPhase = Phase.QUASISTATIC_FORWARD;
                break;
            case STEPS:
                currentPhase = Phase.VOLTAGE_STEPS_HOLD;
                break;
            case COASTDOWN:
                currentPhase = Phase.COASTDOWN_SPINUP;
                break;
        }
        
        stepIndex = 0;
        phaseStartTime = Timer.getFPGATimestamp();
        consecutiveStatorViolations = 0;
        stopDrivetrain();
        publishStatus("Starting routine " + routineToRun.name());
    }

    @Override
    public void execute() {
        if (!safetyCheck()) return;

        switch (currentPhase) {
            case QUASISTATIC_FORWARD:
                executeQuasistaticForward();
                break;
            case VOLTAGE_STEPS_HOLD:
                executeVoltageStepHold();
                break;
            case VOLTAGE_STEPS_SETTLE:
                executeVoltageStepSettle();
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

        applyVoltage(voltage);
        logDataPoint(PHASE_ID_QUASISTATIC, CMD_VOLTAGE, voltage);

        if (voltage >= QS_MAX_VOLTAGE) {
            transitionTo(Phase.SETTLING, "Settling after quasistatic");
            stopDrivetrain();
        }
    }

    private void executeVoltageStepHold() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double voltage = VOLTAGE_STEPS[stepIndex];

        applyVoltage(voltage);
        logDataPoint(PHASE_ID_VOLTAGE_STEPS, CMD_VOLTAGE, voltage);

        if (elapsed >= VOLTAGE_STEP_HOLD_SECONDS) {
            stopDrivetrain();
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
                transitionTo(Phase.SETTLING, "Settling after voltage steps");
                stopDrivetrain();
            } else {
                transitionTo(Phase.VOLTAGE_STEPS_HOLD, "Voltage step " + (stepIndex + 1) + "/" + VOLTAGE_STEPS.length);
            }
        }
    }

    private void executeCoastdownSpinup() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;

        applyVoltage(COASTDOWN_SPINUP_VOLTAGE);
        logDataPoint(PHASE_ID_COASTDOWN, CMD_VOLTAGE, COASTDOWN_SPINUP_VOLTAGE);

        if (elapsed >= COASTDOWN_SPINUP_SECONDS) {
            applyNeutral();
            transitionTo(Phase.COASTDOWN, "Coast-down recording");
        }
    }

    private void executeCoastdown() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(drivetrain.getAverageSteerVelocity());

        logDataPoint(PHASE_ID_COASTDOWN, CMD_NONE, 0.0);

        if (velocityRPS < COASTDOWN_MIN_VELOCITY_RPS || elapsed >= COASTDOWN_TIMEOUT_SECONDS) {
            transitionTo(Phase.SETTLING, "Settling after coast-down");
            stopDrivetrain();
        }
    }

    private void executeSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityRPS = Math.abs(drivetrain.getAverageSteerVelocity());

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
        drivetrain.setControl(steerCharacterization.withVolts(voltage));
    }

    private void stopDrivetrain() {
        drivetrain.setControl(brakeRequest);
    }
    
    private void applyNeutral() {
        drivetrain.setControl(steerCharacterization.withVolts(0.0));
    }

    private boolean safetyCheck() {
        double velocityRPS = Math.abs(drivetrain.getAverageSteerVelocity());
        double statorCurrent = Math.abs(drivetrain.getAverageSteerStatorCurrent());

        if (velocityRPS > MAX_VELOCITY_RPS) {
            System.err.println("STEER SYSID: Velocity exceeded " + MAX_VELOCITY_RPS + " rps. Aborting.");
            cancel();
            return false;
        }

        if (statorCurrent > STATOR_CURRENT_ABORT) {
            consecutiveStatorViolations++;
            if (consecutiveStatorViolations >= MAX_STATOR_VIOLATIONS) {
                System.err.println("STEER SYSID: Stator current exceeded " + STATOR_CURRENT_ABORT + "A. Aborting.");
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
        double velocityRPS = drivetrain.getAverageSteerVelocity();
        double positionRot = drivetrain.getAverageSteerPosition();
        
        double motorVoltage = drivetrain.getAverageSteerVoltage();
        double statorCurrent = drivetrain.getAverageSteerStatorCurrent();
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
        System.out.println("SteerSysId: " + description);
    }

    private void publishStatus(String status) {
        if (PREFIX == null) return;
        SmartDashboard.putString(PREFIX + "Status", status);
        SmartDashboard.putString(PREFIX + "Routine", routineToRun.name());
        SmartDashboard.putString(PREFIX + "Phase", currentPhase.name());
    }

    @Override
    public void end(boolean interrupted) {
        stopDrivetrain();
        logger.close();
        if (interrupted) {
            publishStatus("CANCELLED - partial data saved");
            System.out.println("=== STEER SYSID CANCELLED ===");
        } else {
            publishStatus("COMPLETE - data saved");
            System.out.println("=== STEER SYSID COMPLETE ===");
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }
}
