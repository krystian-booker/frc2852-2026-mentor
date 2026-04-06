package frc.robot.subsystems.intakeactuator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeActuatorConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeActuator extends SubsystemBase {
  private final IntakeActuatorIO io;
  private final IntakeActuatorIOInputsAutoLogged inputs = new IntakeActuatorIOInputsAutoLogged();

  private final Alert disconnected =
      new Alert("IntakeActuator motor disconnected!", AlertType.kWarning);

  public IntakeActuator(IntakeActuatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeActuator", inputs);

    disconnected.set(!inputs.connected);
  }

  public void driveExtend() {
    io.setPosition(IntakeActuatorConstants.EXTENDED_POSITION);
  }

  public void driveRetractAg() {
    io.setPosition(IntakeActuatorConstants.RETRACTED_POSITION_AG);
  }

  public void driveRetract() {
    io.setPosition(IntakeActuatorConstants.RETRACTED_POSITION);
  }

  public void driveExtendOpenLoop() {
    io.setDutyCycle(IntakeActuatorConstants.EXTEND_DUTY_CYCLE);
  }

  public void driveRetractOpenLoop() {
    io.setDutyCycle(IntakeActuatorConstants.RETRACT_DUTY_CYCLE);
  }

  public boolean isExtended() {
    return Math.abs(inputs.positionRotations - IntakeActuatorConstants.EXTENDED_POSITION)
        <= IntakeActuatorConstants.POSITION_TOLERANCE_ROTATIONS;
  }

  public boolean isRetracted() {
    return Math.abs(inputs.positionRotations - IntakeActuatorConstants.RETRACTED_POSITION)
        <= IntakeActuatorConstants.POSITION_TOLERANCE_ROTATIONS;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  public void stop() {
    io.stop();
  }

  // Imperative agitation API

  private boolean agitateRetract;
  private long agitateTimer;

  public void resetAgitate() {
    agitateRetract = true;
    agitateTimer = System.currentTimeMillis();
  }

  public void runAgitate() {
    double timeout =
        agitateRetract
            ? IntakeActuatorConstants.AGITATE_RETRACT_SECONDS
            : IntakeActuatorConstants.AGITATE_EXTEND_SECONDS;
    if (System.currentTimeMillis() - agitateTimer >= timeout * 1000) {
      agitateRetract = !agitateRetract;
      agitateTimer = System.currentTimeMillis();
    }
    if (agitateRetract) {
      driveRetractAg();
    } else {
      driveExtend();
    }
  }

  // Commands

  public Command extend() {
    return runOnce(this::driveExtend).withName("IntakeActuator.extend");
  }

  public Command retract() {
    return runOnce(this::driveRetract).withName("IntakeActuator.retract");
  }

  public Command agitate() {
    return runOnce(this::resetAgitate)
        .andThen(run(this::runAgitate))
        .finallyDo(this::driveExtend)
        .withName("IntakeActuator.agitate");
  }
}
