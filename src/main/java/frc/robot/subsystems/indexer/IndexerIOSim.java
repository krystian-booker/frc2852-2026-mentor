package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getNeoVortex(1);

  private final DCMotorSim independentSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR_MODEL, 0.005, 1.0), MOTOR_MODEL);
  private final DCMotorSim groupSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR_MODEL, 0.005, 1.0), MOTOR_MODEL);

  private double independentOutput = 0.0;
  private double groupOutput = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    independentSim.setInputVoltage(MathUtil.clamp(independentOutput * 12.0, -12.0, 12.0));
    groupSim.setInputVoltage(MathUtil.clamp(groupOutput * 12.0, -12.0, 12.0));
    independentSim.update(0.02);
    groupSim.update(0.02);

    inputs.independentConnected = true;
    inputs.groupLeaderConnected = true;
    inputs.groupFollowerConnected = true;
    inputs.independentVelocityRPS = independentSim.getAngularVelocityRPM() / 60.0;
    inputs.independentOutputCurrent = Math.abs(independentSim.getCurrentDrawAmps());
    inputs.independentAppliedOutput = independentOutput;
  }

  @Override
  public void setIndependentDutyCycle(double output) {
    independentOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setGroupDutyCycle(double output) {
    groupOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setAllDutyCycle(double output) {
    independentOutput = MathUtil.clamp(output, -1.0, 1.0);
    groupOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void stop() {
    independentOutput = 0.0;
    groupOutput = 0.0;
  }
}
