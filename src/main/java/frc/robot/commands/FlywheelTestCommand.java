package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.util.DiagnosticLogger;

public class FlywheelTestCommand extends Command {

    private final Flywheel flywheel;
    private final boolean useBangBang;
    private final double targetRPM;
    
    private final DiagnosticLogger logger;

    public FlywheelTestCommand(Flywheel flywheel, boolean useBangBang, double targetRPM) {
        this.flywheel = flywheel;
        this.useBangBang = useBangBang;
        this.targetRPM = targetRPM;
        
        String prefix = useBangBang ? "flywheel_bangbang_test" : "flywheel_pid_test";
        this.logger = new DiagnosticLogger(prefix, new String[]{
            "timestamp", 
            "target_rpm", 
            "current_rpm", 
            "motor_voltage",
            "stator_current"
        });
        
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        logger.open();
    }

    @Override
    public void execute() {
        if (useBangBang) {
            flywheel.setBangBangVelocity(targetRPM);
        } else {
            flywheel.setVelocity(targetRPM);
        }

        if (logger.isOpen()) {
            logger.logRow(
                Timer.getFPGATimestamp(),
                targetRPM,
                flywheel.getCurrentVelocityRPM(),
                flywheel.getMotorVoltage(),
                flywheel.getStatorCurrent()
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        logger.close();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted by releasing the button
        return false;
    }
}
