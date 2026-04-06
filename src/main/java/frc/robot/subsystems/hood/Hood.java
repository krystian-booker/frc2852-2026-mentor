package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Mechanism3d;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Alert disconnected = new Alert("Hood motor disconnected!", AlertType.kWarning);

  // Tunable PID and Motion Magic gains
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", HoodConstants.S);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", HoodConstants.V);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA", HoodConstants.A);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", HoodConstants.G);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", HoodConstants.P);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", HoodConstants.I);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", HoodConstants.D);
  private final LoggedTunableNumber cruiseVel =
      new LoggedTunableNumber("Hood/CruiseVel", HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY);
  private final LoggedTunableNumber accel =
      new LoggedTunableNumber("Hood/Accel", HoodConstants.MOTION_MAGIC_ACCELERATION);
  private final LoggedTunableNumber jerk =
      new LoggedTunableNumber("Hood/Jerk", HoodConstants.MOTION_MAGIC_JERK);
  private final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber(
          "Hood/PositionToleranceDeg", HoodConstants.POSITION_TOLERANCE_DEGREES);

  private double targetPositionDegrees = 0.0;
  private boolean isHomed = false;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    disconnected.set(!inputs.connected);

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

    Logger.recordOutput("Hood/TargetDegrees", targetPositionDegrees);
    Logger.recordOutput("Hood/AtPosition", atPosition());
    Logger.recordOutput("Hood/IsHomed", isHomed);

    Mechanism3d.getInstance().setHoodAngle(inputs.positionDegrees);

    Robot.batteryLogger.reportCurrentUsage("Shooter/Hood", false, inputs.statorCurrentAmps);
  }

  public void setPosition(double degrees) {
    targetPositionDegrees =
        MathUtil.clamp(
            degrees, HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES);
    io.setPosition(targetPositionDegrees);
  }

  public boolean atPosition() {
    return Math.abs(inputs.positionDegrees - targetPositionDegrees) < positionTolerance.get();
  }

  public double getCurrentPositionDegrees() {
    return inputs.positionDegrees;
  }

  public double getTargetPositionDegrees() {
    return targetPositionDegrees;
  }

  public void setNeutral() {
    io.stop();
  }

  public void testDirectionPositive() {
    io.setVoltage(1.0);
  }

  public void testDirectionNegative() {
    io.setVoltage(-1.0);
  }

  public void nudge(double deltaDegrees) {
    setPosition(inputs.positionDegrees + deltaDegrees);
  }

  public void applyVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }

  public double getStatorCurrent() {
    return inputs.statorCurrentAmps;
  }

  public double getMotorVoltage() {
    return inputs.appliedVolts;
  }

  public boolean isHomed() {
    return isHomed;
  }

  /** Drives hood to reverse hard stop, detects stall via stator current, then zeroes encoder. */
  public Command zeroHoodCommand() {
    Timer homingTimer = new Timer();
    int[] stallCount = {0};

    return Commands.sequence(
            runOnce(
                () -> {
                  isHomed = false;
                  stallCount[0] = 0;
                  io.setSoftLimitsEnabled(true, false);
                  homingTimer.restart();
                  io.setVoltage(HoodConstants.HOMING_VOLTAGE);
                }),
            Commands.idle(this)
                .until(
                    () -> {
                      double current = Math.abs(inputs.statorCurrentAmps);
                      if (homingTimer.hasElapsed(HoodConstants.HOMING_TIMEOUT_SECONDS)) {
                        return true;
                      }
                      if (!homingTimer.hasElapsed(
                          HoodConstants.HOMING_STALL_DETECTION_DELAY_SECONDS)) {
                        return false;
                      }
                      if (current >= HoodConstants.HOMING_STALL_CURRENT_THRESHOLD_AMPS) {
                        stallCount[0]++;
                      } else {
                        stallCount[0] = 0;
                      }
                      return stallCount[0] >= HoodConstants.HOMING_STALL_SAMPLE_COUNT;
                    }),
            runOnce(
                () -> {
                  io.stop();
                  if (stallCount[0] >= HoodConstants.HOMING_STALL_SAMPLE_COUNT) {
                    io.zeroEncoder();
                    targetPositionDegrees = 0.0;
                    isHomed = true;
                  } else {
                    System.err.println(
                        "Hood homing timed out without stall detection -- NOT zeroed.");
                  }
                  io.setSoftLimitsEnabled(true, true);
                }))
        .finallyDo(
            () -> {
              io.stop();
              io.setSoftLimitsEnabled(true, true);
            })
        .withName("Hood.zeroHood");
  }
}
