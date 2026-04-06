package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final Alert leaderDisconnected =
      new Alert("Flywheel leader motor disconnected!", AlertType.kWarning);
  private final Alert followerDisconnected =
      new Alert("Flywheel follower motor disconnected!", AlertType.kWarning);

  private double targetVelocityRPM = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    leaderDisconnected.set(!inputs.leaderConnected);
    followerDisconnected.set(!inputs.followerConnected);

    Logger.recordOutput("Flywheel/TargetRPM", targetVelocityRPM);
    Logger.recordOutput("Flywheel/AtSetpoint", atSetpoint());
  }

  public void setVelocity(double rpm) {
    targetVelocityRPM = Math.max(0, rpm);
    if (targetVelocityRPM == 0.0) {
      io.stop();
    } else {
      io.setVelocity(targetVelocityRPM / 60.0);
    }
  }

  public void setBangBangVelocity(double rpm) {
    targetVelocityRPM = Math.max(0, rpm);
    if (targetVelocityRPM == 0.0) {
      io.stop();
    } else {
      if (inputs.velocityRPM < targetVelocityRPM) {
        io.setVoltage(12.0);
      } else {
        io.setVoltage(0.0);
      }
    }
  }

  public boolean atSetpoint() {
    if (targetVelocityRPM == 0.0) {
      return Math.abs(inputs.velocityRPM) < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
    }
    return Math.abs(inputs.velocityRPM - targetVelocityRPM)
        < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  public double getCurrentVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }

  public double getStatorCurrent() {
    return inputs.statorCurrentAmps;
  }

  public double getSupplyVoltage() {
    return inputs.supplyVoltage;
  }

  public void applyCurrent(double amps) {
    io.setCurrent(amps);
  }

  public void applyVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    targetVelocityRPM = 0.0;
    io.stop();
  }

  public void applyTuningConfig(double kS, double kV, double kA, double kP, double kI, double kD) {
    io.setPID(kS, kV, kA, kP, kI, kD);
  }

  public void restoreDefaultConfig() {
    io.setPID(
        FlywheelConstants.S,
        FlywheelConstants.V,
        FlywheelConstants.A,
        FlywheelConstants.P,
        FlywheelConstants.I,
        FlywheelConstants.D);
  }
}
