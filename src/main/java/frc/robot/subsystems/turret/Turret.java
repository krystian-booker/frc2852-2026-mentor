package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingCalculator.AimingResult;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert motorDisconnected =
      new Alert("Turret motor disconnected!", AlertType.kWarning);
  private final Alert canCoderDisconnected =
      new Alert("Turret CANcoder disconnected!", AlertType.kWarning);

  private double targetPositionDegrees = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    motorDisconnected.set(!inputs.motorConnected);
    canCoderDisconnected.set(!inputs.canCoderConnected);

    Logger.recordOutput("Turret/TargetDegrees", targetPositionDegrees);
    Logger.recordOutput("Turret/PositionDegrees", getPositionDegrees());
    Logger.recordOutput("Turret/AtPosition", isAtPosition());
  }

  public void setPosition(double aimDegrees) {
    // Wrap into turret range before clamping
    if (aimDegrees > TurretConstants.MAX_POSITION_DEGREES) {
      aimDegrees -= 360.0;
    } else if (aimDegrees < TurretConstants.MIN_POSITION_DEGREES) {
      aimDegrees += 360.0;
    }
    targetPositionDegrees =
        Math.max(
            TurretConstants.MIN_POSITION_DEGREES,
            Math.min(TurretConstants.MAX_POSITION_DEGREES, aimDegrees));

    // Convert aiming degrees to encoder degrees
    double encoderDegrees =
        targetPositionDegrees + TurretConstants.FORWARD_ENCODER_POSITION_DEGREES;
    io.setPosition(encoderDegrees);
  }

  public void stop() {
    io.stop();
    targetPositionDegrees = getPositionDegrees();
  }

  public boolean isAtPosition() {
    return Math.abs(getPositionDegrees() - targetPositionDegrees)
        < TurretConstants.POSITION_TOLERANCE_DEGREES;
  }

  public boolean isAtPosition(double degrees) {
    return Math.abs(getPositionDegrees() - degrees) < TurretConstants.POSITION_TOLERANCE_DEGREES;
  }

  public double getPositionDegrees() {
    return inputs.motorPositionDegrees - TurretConstants.FORWARD_ENCODER_POSITION_DEGREES;
  }

  public double getCANCoderPositionDegrees() {
    return inputs.canCoderPositionDegrees;
  }

  public double getEncoderDegrees() {
    return inputs.motorPositionDegrees;
  }

  public double getTargetPositionDegrees() {
    return targetPositionDegrees;
  }

  public Command aimAtTargetCommand(AimingCalculator calculator) {
    return run(() -> {
          AimingResult result = calculator.calculate();
          if (result.isReachable()) {
            setPosition(result.turretAngleDegrees());
          }
        })
        .withName("TurretAimAtTarget");
  }

  public Command fieldHoldCommand(DoubleSupplier robotHeadingSupplier) {
    double[] fieldTarget = new double[1];
    return runOnce(
            () -> {
              fieldTarget[0] = getPositionDegrees() + robotHeadingSupplier.getAsDouble();
            })
        .andThen(
            run(
                () -> {
                  double turretAngle = fieldTarget[0] - robotHeadingSupplier.getAsDouble();
                  turretAngle = ((turretAngle % 360.0) + 540.0) % 360.0 - 180.0;
                  if (turretAngle > TurretConstants.MAX_POSITION_DEGREES) {
                    turretAngle -= 360.0;
                  }
                  setPosition(turretAngle);
                }))
        .withName("TurretFieldHold");
  }

  public void testDirectionPositive() {
    io.setVoltage(1.0);
  }

  public void testDirectionNegative() {
    io.setVoltage(-1.0);
  }

  public void applyVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void disableSoftLimits() {
    io.setSoftLimitsEnabled(false);
  }

  public void enableSoftLimits() {
    io.setSoftLimitsEnabled(true);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }

  public double getMotorVoltage() {
    return inputs.appliedVolts;
  }

  public double getStatorCurrent() {
    return inputs.statorCurrentAmps;
  }

  public void nudge(double deltaDegrees) {
    setPosition(getPositionDegrees() + deltaDegrees);
  }

  public void applyTuningConfig(
      double kS,
      double kV,
      double kA,
      double kG,
      double kP,
      double kI,
      double kD,
      double cruiseVelocity,
      double acceleration,
      double jerk) {
    io.setPID(kS, kV, kA, kG, kP, kI, kD, cruiseVelocity, acceleration, jerk);
  }

  public void restoreDefaultConfig() {
    io.setPID(
        TurretConstants.S,
        TurretConstants.V,
        TurretConstants.A,
        TurretConstants.G,
        TurretConstants.P,
        TurretConstants.I,
        TurretConstants.D,
        TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY,
        TurretConstants.MOTION_MAGIC_ACCELERATION,
        TurretConstants.MOTION_MAGIC_JERK);
  }
}
