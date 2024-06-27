package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

public class ArmSimWrapper {
  private final static double UPDATE_TIME = 0.020;

  private final TalonFXSimState m_armSimState;
  private final TalonFXSimState m_wristSimState;

  private final SingleJointedArmSim m_armSim;
  private final SingleJointedArmSim m_wristSim;

  public ArmSimWrapper(TalonFX armMotor, TalonFX wristMotor) {
    m_armSimState = armMotor.getSimState();
    m_wristSimState = wristMotor.getSimState();

    m_armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2), ArmConstants.ARM_SENSOR_MECHANISM_RATIO, 0.0060620304,
            ArmConstants.ARM_LENGTH_METERS, 0, Units.degreesToRadians(180), true, Units.degreesToRadians(45));
    m_wristSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2), ArmConstants.WRIST_SENSOR_MECHANISM_RATIO, 0.0060620304,
            ArmConstants.WRIST_LENGTH_METERS, 0.0, Units.degreesToRadians(180), true, Units.degreesToRadians(45));
  }

  public void update() {
    m_armSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_wristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_armSim.setInputVoltage(m_armSimState.getMotorVoltage());
    m_wristSim.setInputVoltage(m_wristSimState.getMotorVoltage());

    m_armSim.update(UPDATE_TIME);
    m_wristSim.update(UPDATE_TIME);

    m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
    m_wristSimState.setRawRotorPosition(Units.radiansToRotations(m_wristSim.getAngleRads()));

    m_armSimState.setRotorVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec()));
    m_wristSimState.setRotorVelocity(Units.radiansToRotations(m_wristSim.getVelocityRadPerSec()));
  }
}
