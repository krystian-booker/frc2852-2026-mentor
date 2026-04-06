package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Alert disconnected = new Alert("Hood motor disconnected!", AlertType.kWarning);

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

    Logger.recordOutput("Hood/TargetDegrees", targetPositionDegrees);
    Logger.recordOutput("Hood/AtPosition", atPosition());
    Logger.recordOutput("Hood/IsHomed", isHomed);
  }

  public void setPosition(double degrees) {
    targetPositionDegrees =
        MathUtil.clamp(
            degrees, HoodConstants.MIN_POSITION_DEGREES, HoodConstants.MAX_POSITION_DEGREES);
    io.setPosition(targetPositionDegrees);
  }

  public boolean atPosition() {
    return Math.abs(inputs.positionDegrees - targetPositionDegrees)
        < HoodConstants.POSITION_TOLERANCE_DEGREES;
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

  public void applyTuningConfig(
      double kS,
      double kV,
      double kA,
      double kG,
      double kP,
      double kI,
      double kD,
      double cruiseVelDegS,
      double accelDegS2,
      double jerkDegS3) {
    io.setPID(kS, kV, kA, kG, kP, kI, kD, cruiseVelDegS, accelDegS2, jerkDegS3);
  }

  public void restoreDefaultConfig() {
    io.setPID(
        HoodConstants.S,
        HoodConstants.V,
        HoodConstants.A,
        HoodConstants.G,
        HoodConstants.P,
        HoodConstants.I,
        HoodConstants.D,
        HoodConstants.MOTION_MAGIC_CRUISE_VELOCITY,
        HoodConstants.MOTION_MAGIC_ACCELERATION,
        HoodConstants.MOTION_MAGIC_JERK);
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
                  io.setSoftLimitsEnabled(true, false); // Disable reverse soft limit for homing
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
