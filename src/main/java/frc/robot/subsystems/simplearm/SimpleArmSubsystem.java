package frc.robot.subsystems.simplearm;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmVisualizer;
import monologue.Logged;

public class SimpleArmSubsystem extends SubsystemBase implements Logged {
  TalonFX m_armMaster = new TalonFX(Constants.ArmConstants.ARM_MASTER_ID);
  TalonFX m_armFollower = new TalonFX(Constants.ArmConstants.ARM_FOLLOWER_ID);
  TalonFX m_wristMaster = new TalonFX(Constants.ArmConstants.WRIST_MASTER_ID);
  TalonFX m_wristFollower = new TalonFX(Constants.ArmConstants.WRIST_FOLLOWER_ID);

  CANcoder m_armEncoder = new CANcoder(Constants.ArmConstants.ARM_ENCODER_ID, Constants.CANBUS_NAME);
  CANcoder m_wristEncoder = new CANcoder(Constants.ArmConstants.WRIST_ENCODER_ID, Constants.CANBUS_NAME);

  PositionVoltage m_mm = new PositionVoltage(0.0).withSlot(0);

  ArmVisualizer realArm = new ArmVisualizer("Current Pose");
  ArmVisualizer setpointArm = new ArmVisualizer("Setpoint Pose");

  double desiredArmPose = 0.0;
  double desiredWristPose = 0.0;

  public SimpleArmSubsystem() {
    configMotors();
  }

  @Override
  public void periodic() {
    log("Wrist Applied Voltage", m_wristMaster.get()  * 12.0);
    log("Arm Applied Voltage", m_armMaster.get()  * 12.0);

    realArm.update(m_armMaster.getPosition().getValueAsDouble() * 360,
        m_wristMaster.getPosition().getValueAsDouble() * 360);
    setpointArm.update(desiredArmPose, desiredWristPose);
  }

  public void setArmPosition(double armAngleDegrees, double wristAngleDegrees) {
    m_armMaster.setControl(m_mm.withPosition(Units.degreesToRotations(armAngleDegrees)));
    m_wristMaster.setControl(m_mm.withPosition(Units.degreesToRotations(wristAngleDegrees)));

    m_armFollower.setControl(new Follower(m_armMaster.getDeviceID(), false));
    m_wristFollower.setControl(new Follower(m_wristMaster.getDeviceID(), false));

    desiredArmPose = armAngleDegrees;
    desiredWristPose = wristAngleDegrees;
  }

  public Command setArmPositionFactory(double armDegrees, double wristDegrees) {
    return runOnce(() -> setArmPosition(armDegrees, wristDegrees));
  }

  public void resetPosition() {
    m_armMaster.setPosition(
        (m_armEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(Constants.ArmConstants.OFFSET_NUDGE))
            / Constants.ArmConstants.ARM_CANCODER_MECHANISM_RATIO);
    m_armFollower.setPosition(m_armMaster.getPosition().getValueAsDouble());

    m_wristMaster.setPosition(
        (m_wristEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(Constants.ArmConstants.OFFSET_NUDGE))
            / Constants.ArmConstants.WRIST_CANCODER_MECHANISM_RATIO);
    m_wristFollower.setPosition(m_wristMaster.getPosition().getValueAsDouble());

    if (Utils.isSimulation()) {
      m_armMaster.setPosition(Units.degreesToRotations(0.0));
      m_armFollower.setPosition(Units.degreesToRotations(0.0));

      m_wristMaster.setPosition(Units.degreesToRotations(45.0));
      m_wristFollower.setPosition(Units.degreesToRotations(45.0));
    }
  }

  public void configMotors() {
    // Arm Configuration
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimit = 40;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    armConfig.Feedback.SensorToMechanismRatio = Constants.ArmConstants.ARM_SENSOR_MECHANISM_RATIO;

    armConfig.Slot0.kP = Constants.ArmConstants.ARM_KP;

    m_armMaster.getConfigurator().apply(armConfig);
    m_armFollower.getConfigurator().apply(armConfig);

    // Wrist Configuration
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.Feedback.SensorToMechanismRatio = Constants.ArmConstants.WRIST_SENSOR_MECHANISM_RATIO;

    wristConfig.Slot0.kP = Constants.ArmConstants.WRIST_KP;

    m_wristMaster.getConfigurator().apply(wristConfig);
    m_wristFollower.getConfigurator().apply(wristConfig);

    // Encoder Configuration
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.ARM_OFFSET;

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.WRIST_OFFSET;

    m_armEncoder.getConfigurator().apply(armEncoderConfig);
    m_wristEncoder.getConfigurator().apply(wristEncoderConfig);

//    BaseStatusSignal.setUpdateFrequencyForAll(
//            100,
//            m_armMaster.getPosition(),
//            m_wristMaster.getPosition()
//    );

    resetPosition();
  }
}

