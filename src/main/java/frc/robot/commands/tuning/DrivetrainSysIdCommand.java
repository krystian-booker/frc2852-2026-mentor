package frc.robot.commands.tuning;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DiagnosticLogger;

/**
 * Drivetrain system identification command for MATLAB. Refactored into isolated test routines
 * so that each can be executed manually to fit securely in a 7-meter space.
 * All data is logged to CSV via DiagnosticLogger for import into MATLAB.
 */
public class DrivetrainSysIdCommand extends Command {

    public enum Routine {
        QUASISTATIC,
        STEPS,
        COASTDOWN
    }

    // Safety limits for drivetrain (shortened for 7m constraint)
    private static final double MAX_VELOCITY_MPS = 4.0;
    private static final double STATOR_CURRENT_ABORT = 120.0;
    private static final int MAX_STATOR_VIOLATIONS = 20;

    // Quasistatic ramp parameters
    private static final double QS_RAMP_RATE_V_PER_SEC = 0.5; // 0→4V in 8s
    private static final double QS_MAX_VOLTAGE = 4.0;

    // Voltage step test parameters
    private static final double[] VOLTAGE_STEPS = { 2.0, 3.0, 4.0 };
    private static final double VOLTAGE_STEP_HOLD_SECONDS = 1.5;
    private static final double VOLTAGE_STEP_SETTLE_SECONDS = 1.0;

    // Coast-down parameters
    private static final double COASTDOWN_SPINUP_VOLTAGE = 4.0;
    private static final double COASTDOWN_SPINUP_SECONDS = 1.5;
    private static final double COASTDOWN_MIN_VELOCITY_MPS = 0.05;
    private static final double COASTDOWN_TIMEOUT_SECONDS = 8.0;

    // Settle parameters
    private static final double SETTLE_VELOCITY_THRESHOLD_MPS = 0.05;
    private static final double SETTLE_TIMEOUT_SECONDS = 5.0;

    // Phase IDs for CSV (matches MATLAB script)
    private static final double PHASE_ID_QUASISTATIC = 0.0;
    private static final double PHASE_ID_VOLTAGE_STEPS = 1.0;
    private static final double PHASE_ID_COASTDOWN = 2.0;

    // Command type IDs for CSV
    private static final double CMD_VOLTAGE = 0.0;
    private static final double CMD_NONE = 2.0;

    private static final String PREFIX = "DrivetrainSysId/";

    private final CommandSwerveDrivetrain drivetrain;
    private final DiagnosticLogger logger;
    private final Routine routineToRun;

    // Characterization request specific to Phoenix 6 Swerve
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
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

    public DrivetrainSysIdCommand(CommandSwerveDrivetrain drivetrain, Routine routine) {
        this.drivetrain = drivetrain;
        this.routineToRun = routine;
        
        String logPrefix = "drivetrain_sysid_";
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
                "velocity_mps",
                "distance_m",
                "drive_voltage",
                "drive_stator_current",
                "supply_voltage"
        });
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("=== DRIVETRAIN SYSID STARTING (" + routineToRun.name() + ") ===");
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
            applyNeutral(); // Let it coast instead of braking
            transitionTo(Phase.COASTDOWN, "Coast-down recording");
        }
    }

    private void executeCoastdown() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityMPS = Math.abs(drivetrain.getAverageDriveVelocity());

        logDataPoint(PHASE_ID_COASTDOWN, CMD_NONE, 0.0);

        if (velocityMPS < COASTDOWN_MIN_VELOCITY_MPS || elapsed >= COASTDOWN_TIMEOUT_SECONDS) {
            transitionTo(Phase.SETTLING, "Settling after coast-down");
            stopDrivetrain();
        }
    }

    private void executeSettle() {
        double elapsed = Timer.getFPGATimestamp() - phaseStartTime;
        double velocityMPS = Math.abs(drivetrain.getAverageDriveVelocity());

        double currentPhaseId = 0;
        if (routineToRun == Routine.QUASISTATIC) currentPhaseId = PHASE_ID_QUASISTATIC;
        else if (routineToRun == Routine.STEPS) currentPhaseId = PHASE_ID_VOLTAGE_STEPS;
        else if (routineToRun == Routine.COASTDOWN) currentPhaseId = PHASE_ID_COASTDOWN;
        
        logDataPoint(currentPhaseId, CMD_NONE, 0.0); // Stop logging or log zero? Keep logging for full trace.

        if (velocityMPS < SETTLE_VELOCITY_THRESHOLD_MPS || elapsed >= SETTLE_TIMEOUT_SECONDS) {
            transitionTo(Phase.DONE, "Done");
        }
    }

    // --- Helpers ---
    private void applyVoltage(double voltage) {
        drivetrain.setControl(translationCharacterization.withVolts(voltage));
    }

    private void stopDrivetrain() {
        drivetrain.setControl(brakeRequest);
    }
    
    private void applyNeutral() {
        drivetrain.setControl(translationCharacterization.withVolts(0.0));
    }

    private boolean safetyCheck() {
        double velocityMPS = Math.abs(drivetrain.getAverageDriveVelocity());
        double statorCurrent = Math.abs(drivetrain.getAverageStatorCurrent());

        if (velocityMPS > MAX_VELOCITY_MPS) {
            System.err.println("DRIVETRAIN SYSID: Velocity exceeded " + MAX_VELOCITY_MPS + " m/s. Aborting.");
            cancel();
            return false;
        }

        if (statorCurrent > STATOR_CURRENT_ABORT) {
            consecutiveStatorViolations++;
            if (consecutiveStatorViolations >= MAX_STATOR_VIOLATIONS) {
                System.err.println("DRIVETRAIN SYSID: Stator current exceeded " + STATOR_CURRENT_ABORT + "A. Aborting.");
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
        double velocityMPS = drivetrain.getAverageDriveVelocity();
        double distanceM = drivetrain.getState().Pose.getTranslation().getNorm(); 
        
        double motorVoltage = drivetrain.getAverageDriveVoltage();
        double statorCurrent = drivetrain.getAverageStatorCurrent();
        double supplyVoltage = RobotController.getBatteryVoltage();

        logger.logRow(
                timestamp,
                phaseId,
                commandType,
                commandValue,
                velocityMPS,
                distanceM,
                motorVoltage,
                statorCurrent,
                supplyVoltage);
    }

    private void transitionTo(Phase phase, String description) {
        currentPhase = phase;
        phaseStartTime = Timer.getFPGATimestamp();
        publishStatus(description);
        System.out.println("DrivetrainSysId: " + description);
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
            System.out.println("=== DRIVETRAIN SYSID CANCELLED ===");
        } else {
            publishStatus("COMPLETE - data saved");
            System.out.println("=== DRIVETRAIN SYSID COMPLETE ===");
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }
}
