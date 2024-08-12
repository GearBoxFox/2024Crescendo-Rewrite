package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import monologue.Annotations;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;

  private final TalonFX m_kicker;
  private final TalonFX m_intake;
  private final TalonFX m_indexer;

  private final TimeOfFlight m_noteSensor;

  private final PidPropertyPublic m_leftPid;
  private final PidPropertyPublic m_rightPid;

  // sim variables
  private TalonFXSimState m_leftSimState;
  private TalonFXSimState m_rightSimState;

  private FlywheelSim m_leftSim;
  private FlywheelSim m_rightSim;

  @Annotations.Log
  private double leftRPMSetpoint = 0.0;
  @Annotations.Log
  private double rightRPMSetpoint = 0.0;

  public ShooterSubsystem() {
    // Initialize and setup motors
    m_leftShooter =
        new TalonFX(ShooterConstants.LEFT_ID, Constants.CANBUS_NAME);
    m_rightShooter =
        new TalonFX(ShooterConstants.RIGHT_ID, Constants.CANBUS_NAME);

    m_kicker = new TalonFX(ShooterConstants.KICKER_ID, Constants.CANBUS_NAME);
    m_intake = new TalonFX(ShooterConstants.INTAKE_ID, Constants.CANBUS_NAME);
    m_indexer = new TalonFX(ShooterConstants.INDEXER_ID, Constants.CANBUS_NAME);

    m_noteSensor = new TimeOfFlight(ShooterConstants.TOF_ID);
    m_noteSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 25);

    configMotors();

    // Setup PID Controllers
    m_leftPid = new Phoenix6PidPropertyBuilder("Shooter/Left PID",
        true,
        m_leftShooter,
        0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV)
        .build();

    m_rightPid = new Phoenix6PidPropertyBuilder("Shooter/Right PID",
        true,
        m_rightShooter,
        0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV + ShooterConstants.RIGHT_SHOOTER_NUDGE)
        .build();

    // setup flywheel sim
    if (RobotBase.isSimulation()) {
      m_leftSimState = m_leftShooter.getSimState();
      m_rightSimState = m_rightShooter.getSimState();

      m_leftSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.01);
      m_rightSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.01);
    }
  }

  public Command setShooterVelocities(double leftRPM, double rightRPM) {
    return runEnd(() -> {
          leftRPMSetpoint = leftRPM;
          rightRPMSetpoint = rightRPM;
          var control = new VelocityVoltage(0.0)
              .withSlot(0);
          m_leftShooter.setControl(control.withVelocity(leftRPM / 60.0));
          m_rightShooter.setControl(control.withVelocity(rightRPM / 60.0));
        },
        () -> {
          leftRPMSetpoint = 0.0;
          rightRPMSetpoint = 0.0;
          m_leftShooter.set(0.0);
          m_rightShooter.set(0.0);
        });
  }

  public Command intakeCommand(double intakePower, double kickerPower, double timeout) {
    Timer timer = new Timer();
    return runEnd(() -> {
          if (!hasGamePiece()) {
            // if no game piece, intake normally
            m_leftShooter.set(-0.1);
            m_rightShooter.set(-0.1);
            m_intake.set(intakePower);
            m_kicker.set(kickerPower);
            timer.restart();
          } else if (!timer.hasElapsed(timeout)) {
            // when we have a game piece, run backwards for so long
            m_kicker.set(-0.15);
            m_intake.set(0.0);
          } else {
            // stop when finished
            m_leftShooter.set(0.0);
            m_rightShooter.set(0.0);
            m_kicker.set(0.0);
            m_intake.set(0.0);
          }
        },
        () -> {
          // stop when finished
          m_intake.set(0.0);
          m_kicker.set(0.0);
          m_leftShooter.set(0.0);
          m_rightShooter.set(0.0);
        });
  }

  public boolean hasGamePiece() {
    return m_noteSensor.getRange() < 60.0;
  }

  @Override
  public void periodic() {
    // Update PID Controller
    m_leftPid.updateIfChanged();
    m_rightPid.updateIfChanged();

    // Log new values
    log("Left Shooter RPM", m_leftShooter.getVelocity().getValueAsDouble() * 60.0);
    log("Right Shooter RPM", m_rightShooter.getVelocity().getValueAsDouble() * 60.0);
    log("Left Motor RPS", m_leftShooter.getVelocity().getValueAsDouble());
    log("Right Motor RPS", m_rightShooter.getVelocity().getValueAsDouble());

    log("Has game piece", hasGamePiece());
  }

  @Override
  public void simulationPeriodic() {
    // set input voltage from motors
    m_leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_leftSim.setInputVoltage(m_leftSimState.getMotorVoltage());
    m_rightSim.setInputVoltage(m_rightSimState.getMotorVoltage());

    m_leftSim.update(0.02); // assume 20ms sim loop time
    m_rightSim.update(0.02);

    m_leftSimState.setRotorVelocity(
            Units.radiansToRotations(m_leftSim.getAngularVelocityRadPerSec())
    );
    m_rightSimState.setRotorVelocity(
            Units.radiansToRotations(m_rightSim.getAngularVelocityRadPerSec())
    );
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

  public boolean atSetpoint() {
    return Math.abs((m_leftShooter.getVelocity().getValueAsDouble() / 60.0) - leftRPMSetpoint) < 100.0
            && Math.abs((m_rightShooter.getVelocity().getValueAsDouble() / 60.0) - rightRPMSetpoint) < 100.0;
  }

  public void runKicker(boolean runKicker) {
    m_kicker.set(runKicker ? 0.9 : 0.0);
  }

  public Command runShooter(double volts) {
    return runEnd(() -> {
          m_leftShooter.setVoltage(volts);
          m_rightShooter.setVoltage(volts);
        },
        () -> {
          m_leftShooter.setVoltage(0.0);
          m_rightShooter.setVoltage(0.0);
        });
  }
}

