package frc.robot.subsystems.intakeactuator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeActuatorConstants;

public class IntakeActuatorIOSim implements IntakeActuatorIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getNeoVortex(1);

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTOR_MODEL, 0.01, IntakeActuatorConstants.GEAR_RATIO),
          MOTOR_MODEL);

  private final PIDController controller = new PIDController(IntakeActuatorConstants.KP, 0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedOutput = 0.0;

  @Override
  public void updateInputs(IntakeActuatorIOInputs inputs) {
    double volts;
    if (closedLoop) {
      double positionRotations = sim.getAngularPositionRotations();
      volts = MathUtil.clamp(controller.calculate(positionRotations), -1.0, 1.0) * 12.0;
    } else {
      volts = appliedOutput * 12.0;
    }

    sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityRPS = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedOutput = closedLoop ? volts / 12.0 : appliedOutput;
  }

  @Override
  public void setPosition(double positionRotations) {
    closedLoop = true;
    controller.setSetpoint(positionRotations);
  }

  @Override
  public void setDutyCycle(double output) {
    closedLoop = false;
    appliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedOutput = 0.0;
  }

  @Override
  public void zeroEncoder() {
    // No-op in sim
  }
}
