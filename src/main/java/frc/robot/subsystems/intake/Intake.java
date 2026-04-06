package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert leftDisconnected =
      new Alert("Intake left motor disconnected!", AlertType.kWarning);
  private final Alert rightDisconnected =
      new Alert("Intake right motor disconnected!", AlertType.kWarning);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    leftDisconnected.set(!inputs.leftConnected);
    rightDisconnected.set(!inputs.rightConnected);
  }

  public void runIntake() {
    io.setDutyCycle(1.0);
  }

  public void runOuttake() {
    io.setDutyCycle(-1.0);
  }

  public void stop() {
    io.stop();
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }

  public Command runCommand() {
    return run(this::runIntake).finallyDo(this::stop).withName("Intake.run");
  }
}
