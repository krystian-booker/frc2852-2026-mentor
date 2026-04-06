package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR_MODEL, 0.01, 1.0), MOTOR_MODEL);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.leftConnected = true;
    inputs.rightConnected = true;
    inputs.velocityRPS = sim.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void setDutyCycle(double output) {
    appliedVolts = MathUtil.clamp(output, -1.0, 1.0) * 12.0;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
