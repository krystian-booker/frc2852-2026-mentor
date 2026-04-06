package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final Alert leaderDisconnected =
      new Alert("Flywheel leader motor disconnected!", AlertType.kWarning);
  private final Alert followerDisconnected =
      new Alert("Flywheel follower motor disconnected!", AlertType.kWarning);

  // Tunable PID gains
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Flywheel/kS", FlywheelConstants.S);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Flywheel/kV", FlywheelConstants.V);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Flywheel/kA", FlywheelConstants.A);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Flywheel/kP", FlywheelConstants.P);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Flywheel/kI", FlywheelConstants.I);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Flywheel/kD", FlywheelConstants.D);
  private final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber(
          "Flywheel/VelocityToleranceRPM", FlywheelConstants.VELOCITY_TOLERANCE_RPM);

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

    // Auto-apply PID gains when tunable values change
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> io.setPID(values[0], values[1], values[2], values[3], values[4], values[5]),
        kS,
        kV,
        kA,
        kP,
        kI,
        kD);

    Logger.recordOutput("Flywheel/TargetRPM", targetVelocityRPM);
    Logger.recordOutput("Flywheel/AtSetpoint", atSetpoint());

    Robot.batteryLogger.reportCurrentUsage("Shooter/Flywheel", false, inputs.statorCurrentAmps);
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
      return Math.abs(inputs.velocityRPM) < velocityTolerance.get();
    }
    return Math.abs(inputs.velocityRPM - targetVelocityRPM) < velocityTolerance.get();
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
}
