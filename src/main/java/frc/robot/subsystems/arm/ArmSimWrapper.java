package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
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

    m_armSimState.Orientation = ChassisReference.Clockwise_Positive;
    m_wristSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    m_armSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_wristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2), 1.0, 0.0060620304,
            ArmConstants.ARM_LENGTH_METERS, Units.degreesToRadians(ArmConstants.ARM_LOWER_LIMIT), Units.degreesToRadians(180), true, Units.degreesToRadians(45));
    m_wristSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2), 1.0, 0.0060620304,
            ArmConstants.WRIST_LENGTH_METERS, Units.degreesToRadians(ArmConstants.WRIST_LOWER_LIMIT), Units.degreesToRadians(180), true, Units.degreesToRadians(45));
  }

  public void update() {
    m_armSim.setInputVoltage(m_armSimState.getMotorVoltage());
    m_wristSim.setInputVoltage(m_wristSimState.getMotorVoltage());

    m_armSim.update(UPDATE_TIME);
    m_wristSim.update(UPDATE_TIME);

    m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
    m_wristSimState.setRawRotorPosition(Units.radiansToRotations(m_wristSim.getAngleRads()));
  }

  public double getArmPosition() {
    return Units.radiansToRotations(m_armSim.getAngleRads());
  }
}
