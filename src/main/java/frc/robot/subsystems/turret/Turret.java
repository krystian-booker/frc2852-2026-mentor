package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Mechanism3d;
import frc.robot.Robot;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingCalculator.AimingResult;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert motorDisconnected =
      new Alert("Turret motor disconnected!", AlertType.kWarning);
  private final Alert canCoderDisconnected =
      new Alert("Turret CANcoder disconnected!", AlertType.kWarning);

  // Tunable PID and Motion Magic gains
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", TurretConstants.S);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", TurretConstants.V);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", TurretConstants.A);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/kG", TurretConstants.G);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", TurretConstants.P);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", TurretConstants.I);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", TurretConstants.D);
  private final LoggedTunableNumber cruiseVel =
      new LoggedTunableNumber("Turret/CruiseVel", TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY);
  private final LoggedTunableNumber accel =
      new LoggedTunableNumber("Turret/Accel", TurretConstants.MOTION_MAGIC_ACCELERATION);
  private final LoggedTunableNumber jerk =
      new LoggedTunableNumber("Turret/Jerk", TurretConstants.MOTION_MAGIC_JERK);
  private final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber(
          "Turret/PositionToleranceDeg", TurretConstants.POSITION_TOLERANCE_DEGREES);

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

    // Auto-apply gains when tunable values change
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values ->
            io.setPID(
                values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                values[7], values[8], values[9]),
        kS,
        kV,
        kA,
        kG,
        kP,
        kI,
        kD,
        cruiseVel,
        accel,
        jerk);

    Logger.recordOutput("Turret/TargetDegrees", targetPositionDegrees);
    Logger.recordOutput("Turret/PositionDegrees", getPositionDegrees());
    Logger.recordOutput("Turret/AtPosition", isAtPosition());

    Mechanism3d.getInstance().setTurretAngle(getPositionDegrees());

    Robot.batteryLogger.reportCurrentUsage("Shooter/Turret", false, inputs.statorCurrentAmps);
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
    return Math.abs(getPositionDegrees() - targetPositionDegrees) < positionTolerance.get();
  }

  public boolean isAtPosition(double degrees) {
    return Math.abs(getPositionDegrees() - degrees) < positionTolerance.get();
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
}
