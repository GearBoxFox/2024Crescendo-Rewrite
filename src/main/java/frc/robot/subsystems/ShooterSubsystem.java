package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;

  private final TalonFX m_kicker;
  private final TalonFX m_intake;
  private final TalonFX m_indexer;

  private final PidPropertyPublic m_leftPid;
  private final PidPropertyPublic m_rightPid;

  public ShooterSubsystem() {
    // Initialize and setup motors
    m_leftShooter =
        new TalonFX(ShooterConstants.LEFT_ID, Constants.CANBUS_NAME);
    m_rightShooter =
        new TalonFX(ShooterConstants.RIGHT_ID, Constants.CANBUS_NAME);

    m_kicker = new TalonFX(ShooterConstants.KICKER_ID, Constants.CANBUS_NAME);
    m_intake = new TalonFX(ShooterConstants.INTAKE_ID, Constants.CANBUS_NAME);
    m_indexer = new TalonFX(ShooterConstants.INDEXER_ID, Constants.CANBUS_NAME);

    configMotors();

    // Setup PID Controllers
    m_leftPid = new Phoenix6PidPropertyBuilder("Shooter/Left PID",
        false,
        m_leftShooter,
        0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV)
        .build();

    m_rightPid = new Phoenix6PidPropertyBuilder("Shooter/Right PID",
        false,
        m_rightShooter,
        0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV)
        .build();
  }

  public void configMotors() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Left shooter is inverted, all else isn't
    m_leftShooter.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rightShooter.getConfigurator().apply(config);
    m_kicker.getConfigurator().apply(config);
    m_intake.getConfigurator().apply(config);
    m_indexer.getConfigurator().apply(config);
  }
}

