package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import monologue.Annotations;
import monologue.Logged;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase implements Logged {
  public enum ArmState {
    DISABLED,
    SETPOINT,
    AIMBOT,
    TRAJECTORY
  }

  // Motors and encoders on the arm
  private final TalonFX m_armMaster;
  private final TalonFX m_armFollower;
  private final CANcoder m_armEncoder;

  // Motors and encoders on the arm
  private final TalonFX m_wristMaster;
  private final TalonFX m_wristFollower;
  private final CANcoder m_wristEncoder;

  // Motor outputs
  private final PositionVoltage m_pid;
  private final MotionMagicVoltage m_mm;

  private final Follower m_armFollowerRequest;
  private final Follower m_wristFollowerRequest;

  private final PidPropertyPublic m_armProperty;
  private final PidPropertyPublic m_wristProperty;

  // Variables to control arm
  @Annotations.Log
  private double m_desiredArmPoseDegs = 0.0;
  @Annotations.Log
  private double m_desiredWristPoseDegs = 0.0;
  @Annotations.Log
  private ArmState m_desiredState = ArmState.DISABLED;
  private boolean m_disabledBrakeMode = false;

  public ArmSubsystem() {
    // setup and config motors
    m_armMaster = new TalonFX(ArmConstants.ARM_MASTER_ID, Constants.CANBUS_NAME);
    m_armFollower = new TalonFX(ArmConstants.ARM_FOLLOWER_ID, Constants.CANBUS_NAME);
    m_armEncoder = new CANcoder(ArmConstants.ARM_ENCODER_ID, Constants.CANBUS_NAME);

    m_wristMaster = new TalonFX(ArmConstants.WRIST_MASTER_ID, Constants.CANBUS_NAME);
    m_wristFollower = new TalonFX(ArmConstants.WRIST_FOLLOWER_ID, Constants.CANBUS_NAME);
    m_wristEncoder = new CANcoder(ArmConstants.WRIST_ENCODER_ID, Constants.CANBUS_NAME);

    configMotors();

    //set up pid control
    m_pid = new PositionVoltage(0.0)
        .withEnableFOC(true)
        .withSlot(0);
    m_mm = new MotionMagicVoltage(0.0)
        .withEnableFOC(true)
        .withSlot(0);

    m_armFollowerRequest = new Follower(m_armMaster.getDeviceID(), false);
    m_wristFollowerRequest = new Follower(m_wristMaster.getDeviceID(), false);

    m_armProperty = new Phoenix6PidPropertyBuilder("Arm/Arm Pid", true, m_armMaster, 0)
        .addP(ArmConstants.ARM_KP)
        .addI(ArmConstants.ARM_KI)
        .addD(ArmConstants.ARM_KD)
        .addKG(ArmConstants.ARM_KG, GravityTypeValue.Arm_Cosine)
        .addKS(ArmConstants.ARM_KS)
        .build();

    m_wristProperty = new Phoenix6PidPropertyBuilder("Arm/Wrist Pid", true, m_armMaster, 0)
        .addP(ArmConstants.WRIST_KP)
        .addI(ArmConstants.WRIST_KI)
        .addD(ArmConstants.WRIST_KD)
        .addKG(ArmConstants.WRIST_KG, GravityTypeValue.Arm_Cosine)
        .addKS(ArmConstants.WRIST_KS)
        .build();
  }

  @Override
  public void periodic() {
    // clamp values for PID in between acceptable ranges
    m_desiredWristPoseDegs = m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredWristPoseDegs, ArmConstants.WRIST_LOWER_LIMIT,
            ArmConstants.WRIST_UPPER_LIMIT)
        : m_desiredWristPoseDegs;

    m_desiredArmPoseDegs = m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredArmPoseDegs, ArmConstants.ARM_LOWER_LIMIT,
            ArmConstants.ARM_UPPER_LIMIT)
        : m_desiredArmPoseDegs;

    // if we're disabled go back to hold pose
    if (DriverStation.isDisabled()) {
      m_desiredState = ArmState.DISABLED;
    } else {
      m_disabledBrakeMode = false;
    }

    // check to make sure we're not in manual control
    enableBrakeMode(m_desiredState == ArmState.DISABLED && m_disabledBrakeMode);

    if (m_desiredState != ArmState.DISABLED) {
      // check to see if the wrist is currently too close to the rest of the arm
      double predictedUnderGap = MathUtil.clamp(ArmConstants.WRIST_ARM_GAP
          - (m_desiredArmPoseDegs + m_desiredWristPoseDegs), 0, 180);

      setWristAngle(m_desiredWristPoseDegs + predictedUnderGap);

      // set the arms angle
      setArmAngle(m_desiredArmPoseDegs);
    }
  }

  public void enableBrakeMode(boolean enabled) {
    MotorOutputConfigs config = new MotorOutputConfigs();

    config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Inverted = InvertedValue.Clockwise_Positive;

    m_wristMaster.getConfigurator().apply(config);
    m_wristFollower.getConfigurator().apply(config);

    m_armMaster.getConfigurator().apply(config);
    m_armFollower.getConfigurator().apply(config);
  }

  public void enableBrakeModeFactory(boolean enabled) {
    return runOnce(() -> m_disabledBrakeMode = enabled).ignoringDisable(true);
  }

  public void configMotors() {
    // Arm Configuration
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimit = 40;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    armConfig.Feedback.SensorToMechanismRatio = ArmConstants.ARM_SENSOR_MECHANISM_RATIO;

    m_armMaster.getConfigurator().apply(armConfig);
    m_armFollower.getConfigurator().apply(armConfig);

    // Wrist Configuration
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_SENSOR_MECHANISM_RATIO;

    m_wristMaster.getConfigurator().apply(wristConfig);
    m_wristFollower.getConfigurator().apply(wristConfig);

    // Encoder Configuration
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.ARM_OFFSET;

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.WRIST_OFFSET;

    m_armEncoder.getConfigurator().apply(armEncoderConfig);
    m_wristEncoder.getConfigurator().apply(wristEncoderConfig);
  }
}

