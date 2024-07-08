package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSimWrapper {
  private static final double UPDATE_TIME = 0.020;

  private final TalonFXSimState m_leftSimState;
  private final TalonFXSimState m_rightSimState;

  private final FlywheelSim m_leftSim;
  private final FlywheelSim m_rightSim;

  public ShooterSimWrapper(TalonFX leftMotor, TalonFX rightMotor) {
    m_leftSimState = leftMotor.getSimState();
    m_rightSimState = rightMotor.getSimState();

    m_leftSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.001);
    m_rightSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.001);
  }

  public void update() {
    m_leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_leftSim.setInputVoltage(m_leftSimState.getMotorVoltage());
    m_rightSim.setInputVoltage(m_rightSimState.getMotorVoltage());

    m_leftSim.update(UPDATE_TIME); // assume 20ms sim loop time
    m_rightSim.update(UPDATE_TIME);

    m_leftSimState.setRotorVelocity(
            Units.radiansToRotations(m_leftSim.getAngularVelocityRadPerSec())
    );
    m_rightSimState.setRotorVelocity(
            Units.radiansToRotations(m_rightSim.getAngularVelocityRadPerSec())
    );
  }
}
