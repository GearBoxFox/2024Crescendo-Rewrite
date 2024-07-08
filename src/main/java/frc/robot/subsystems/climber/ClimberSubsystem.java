package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
  private final TalonFX m_leftClimber;
  private final TalonFX m_rightClimber;

  private final PositionVoltage m_posRequest;

  private boolean m_climberLock = false;

  public ClimberSubsystem() {
    m_leftClimber = new TalonFX(ClimberConstants.LEFT_CLIMBER_ID, Constants.CANBUS_NAME);
    m_rightClimber = new TalonFX(ClimberConstants.RIGHT_CLIMBER_ID, Constants.CANBUS_NAME);

    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.Feedback.SensorToMechanismRatio = 36.0;
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfig.Slot0.kP = ClimberConstants.CLIMBER_KP;
    climberConfig.Slot0.kI = ClimberConstants.CLIMBER_KI;
    climberConfig.Slot0.kD = ClimberConstants.CLIMBER_KD;

    m_leftClimber.getConfigurator().apply(climberConfig);

    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rightClimber.getConfigurator().apply(climberConfig);

    m_posRequest = new PositionVoltage(0).withSlot(0)
            .withEnableFOC(m_leftClimber.getIsProLicensed().getValue() && m_rightClimber.getIsProLicensed().getValue());
  }

  public void setPosition(double degrees) {
    m_posRequest.Position = degrees / 360;
    m_leftClimber.setControl(m_posRequest);
    m_rightClimber.setControl(m_posRequest);
  }

  public Command setPositionFactory(double degrees) {
    return runEnd(
            () -> setPosition(degrees),
            () -> {
              m_leftClimber.set(0.0);
              m_rightClimber.set(0.0);
            }
    );
  }

  public boolean getClimberLock() {
    return m_climberLock;
  }

  public Command resetClimberLock() {
    return runOnce(() -> m_climberLock = false);
  }

  @Override
  public void periodic() {
    log("Left Position Degrees", m_leftClimber.getPosition().getValueAsDouble() * 360);
    log("Right Position Degrees", m_rightClimber.getPosition().getValueAsDouble() * 360);
  }
}
