package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HoodConstants;

public class HoodIOSim implements HoodIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          MOTOR_MODEL,
          HoodConstants.GEAR_RATIO,
          0.05, // MOI kg*m^2
          0.15, // arm length meters
          Units.degreesToRadians(HoodConstants.MIN_POSITION_DEGREES),
          Units.degreesToRadians(HoodConstants.MAX_POSITION_DEGREES),
          true, // simulate gravity
          Units.degreesToRadians(0.0)); // starting angle

  private final PIDController controller = new PIDController(8.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (closedLoop) {
      double gravityFF = HoodConstants.G * Math.cos(sim.getAngleRads());
      appliedVolts = controller.calculate(Units.radiansToDegrees(sim.getAngleRads())) + gravityFF;
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPosition(double degrees) {
    closedLoop = true;
    controller.setSetpoint(degrees);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void zeroEncoder() {
    // No-op in sim; the sim tracks absolute position
  }

  @Override
  public void setSoftLimitsEnabled(boolean forward, boolean reverse) {
    // Soft limits are inherently modeled by the SingleJointedArmSim bounds
  }
}
