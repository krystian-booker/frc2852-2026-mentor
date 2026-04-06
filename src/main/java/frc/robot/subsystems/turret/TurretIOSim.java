package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTOR_MODEL, 0.05, TurretConstants.GEAR_RATIO),
          MOTOR_MODEL);

  private final PIDController controller = new PIDController(8.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public TurretIOSim() {
    controller.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoop) {
      double positionDegrees = sim.getAngularPositionRotations() * 360.0;
      appliedVolts = controller.calculate(positionDegrees);
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    double positionDegrees = sim.getAngularPositionRotations() * 360.0;
    inputs.motorConnected = true;
    inputs.canCoderConnected = true;
    inputs.motorPositionDegrees = positionDegrees;
    inputs.canCoderPositionDegrees = positionDegrees; // Mirrors motor in sim
    inputs.velocityRPS = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPosition(double encoderDegrees) {
    closedLoop = true;
    controller.setSetpoint(encoderDegrees);
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
}
