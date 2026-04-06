package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(2);
  private static final double MOI = 0.01; // kg*m^2, estimated flywheel moment of inertia

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTOR_MODEL, MOI, FlywheelConstants.GEAR_RATIO),
          MOTOR_MODEL);

  private final PIDController controller = new PIDController(0.5, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocityRPM() / 60.0);
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.velocityRPS = sim.getAngularVelocityRPM() / 60.0;
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.supplyVoltage = 12.0;
  }

  @Override
  public void setVelocity(double rps) {
    closedLoop = true;
    controller.setSetpoint(rps);
    ffVolts = FlywheelConstants.S * Math.signum(rps) + FlywheelConstants.V * rps;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setCurrent(double amps) {
    closedLoop = false;
    // Approximate voltage from current
    appliedVolts = amps * MOTOR_MODEL.rOhms;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
