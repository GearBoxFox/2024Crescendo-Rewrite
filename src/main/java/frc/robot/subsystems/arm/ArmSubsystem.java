package frc.robot.subsystems.arm;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import lib.utils.AimbotUtils;
import lib.utils.ArmTrajectory;
import monologue.Annotations;
import monologue.Logged;

public class ArmSubsystem extends SubsystemBase implements Logged {
  public enum ArmState {
    DISABLED,
    SETPOINT,
    AIMBOT,
    TRAJECTORY,
    TRAJECTORY_REVERSE,
    STOW
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
  private final DynamicMotionMagicVoltage m_mm;

  private final Follower m_armFollowerRequest;
  private final Follower m_wristFollowerRequest;

  private final PidPropertyPublic m_armProperty;
  private final PidPropertyPublic m_wristProperty;

  // Variables to control arm
  @Annotations.Log
  private double m_desiredArmPoseDegs = 0.0;
  @Annotations.Log
  private double m_desiredArmVelocity = 0.0;
  @Annotations.Log
  private double m_desiredWristPoseDegs = 0.0;
  @Annotations.Log
  private double m_desiredWristVelocity = 0.0;

  private ArmTrajectory m_currentTrajectory = null;
  private final Timer m_trajectoryTimer = new Timer();
  private double m_reverseTimer = 0.0;

  @Annotations.Log
  private ArmState m_desiredState = ArmState.DISABLED;
  private boolean m_disabledBrakeMode = false;

  @Annotations.Log
  private double m_armPoseDegs = 0.0;

  @Annotations.Log
  private double m_wristPoseDegs = 0.0;

  private TalonFXSimState m_armSimState;
  private TalonFXSimState m_wristSimState;

  private SingleJointedArmSim m_armSim;
  private SingleJointedArmSim m_wristSim;

  private final ArmVisualizer m_armViz = new ArmVisualizer("Current Pose");
  private final ArmVisualizer m_setpointViz = new ArmVisualizer("Setpoint Pose");

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
    m_mm = new DynamicMotionMagicVoltage(0.0,
            Units.degreesToRotations(360.0),
            Units.degreesToRotations(360.0 * 0.125), 0.0)
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

    if (Utils.isSimulation()) {
      m_armMaster.getPosition().setUpdateFrequency(1000);
      m_wristMaster.getPosition().setUpdateFrequency(1000);

      m_armSimState = m_armMaster.getSimState();
      m_wristSimState = m_wristMaster.getSimState();

      m_armSimState.Orientation = ChassisReference.Clockwise_Positive;
      m_wristSimState.Orientation = ChassisReference.CounterClockwise_Positive;

      m_armSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      m_wristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      m_armSim = new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(2), ArmConstants.ARM_SENSOR_MECHANISM_RATIO, 0.0060620304,
              ArmConstants.ARM_LENGTH_METERS, Units.degreesToRadians(ArmConstants.ARM_LOWER_LIMIT), Units.degreesToRadians(180), true, Units.degreesToRadians(45));
      m_wristSim = new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(2), ArmConstants.WRIST_SENSOR_MECHANISM_RATIO, 0.0060620304,
              ArmConstants.WRIST_LENGTH_METERS, Units.degreesToRadians(ArmConstants.WRIST_LOWER_LIMIT), Units.degreesToRadians(180), true, Units.degreesToRadians(45));
    }
  }

  @Override
  public void periodic() {
    // update global variables for arm + wrist pose
    m_armPoseDegs = Units.rotationsToDegrees(m_armMaster.getPosition().getValueAsDouble());
    m_wristPoseDegs = Units.rotationsToDegrees(m_wristMaster.getPosition().getValueAsDouble());

    handleState();

    // Reset trajectory timer when not actively running a trajectory
    if (m_desiredState != ArmState.TRAJECTORY && m_desiredState != ArmState.TRAJECTORY_REVERSE) {
      m_trajectoryTimer.stop();
      m_trajectoryTimer.reset();
    }

    if (m_desiredState != ArmState.TRAJECTORY_REVERSE) {
      m_reverseTimer = 0.0;
    }

    // clamp values for PID in between acceptable ranges
    m_desiredWristPoseDegs = m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredWristPoseDegs, ArmConstants.WRIST_LOWER_LIMIT,
            ArmConstants.WRIST_UPPER_LIMIT)
        : m_desiredWristPoseDegs;

    m_desiredArmPoseDegs = m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredArmPoseDegs, ArmConstants.ARM_LOWER_LIMIT,
            ArmConstants.ARM_UPPER_LIMIT)
        : m_desiredArmPoseDegs;

    // clamp values for velocity during trajectories
    m_desiredArmVelocity = MathUtil.clamp(
        m_desiredArmVelocity,
        -ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue(),
        ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue()
    );

    m_desiredWristVelocity = MathUtil.clamp(
        m_desiredWristVelocity,
        -ArmConstants.WRIST_MAX_VELOCITY_DEG_S.getValue(),
        ArmConstants.WRIST_MAX_VELOCITY_DEG_S.getValue()
    );

    // if we're disabled go back to hold pose
    if (DriverStation.isDisabled()) {
      m_desiredState = ArmState.DISABLED;
    } else {
      m_disabledBrakeMode = false;
    }

    // check to make sure we're not in manual control
//    enableBrakeMode(m_desiredState == ArmState.DISABLED && m_disabledBrakeMode);

    if (m_desiredState != ArmState.DISABLED) {
      // check to see if the wrist is currently too close to the rest of the arm
      double predictedUnderGap = MathUtil.clamp(ArmConstants.WRIST_ARM_GAP
          - (m_desiredArmPoseDegs + m_desiredWristPoseDegs), 0, 180);
      m_desiredWristPoseDegs += predictedUnderGap;

      setJointAngles();
    }

    m_armViz.update(m_armPoseDegs, m_wristPoseDegs);
    m_setpointViz.update(m_desiredArmPoseDegs, m_desiredWristPoseDegs);
  }

  public void handleState() {
    switch (m_desiredState) {
      case STOW -> {
        // reset setpoints to be at stow
        m_desiredArmPoseDegs = ArmSetpoints.STOW_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.STOW_SETPOINT.wristAngle();
      }
      case DISABLED -> {
        // when disabled default to current position for setpoints
        m_desiredArmPoseDegs = m_armPoseDegs;
        m_desiredWristPoseDegs = m_wristPoseDegs;
      }
      case TRAJECTORY -> {
        // do we have a trajectory?
        if (m_currentTrajectory != null) {
          // if timer has started (loop time is 0.02 seconds
          if (!m_trajectoryTimer.hasElapsed(0.01)) {
            m_trajectoryTimer.restart();
          }
          // get the current state in the trajectory
          ArmTrajectory.ArmTrajectoryState state =
              m_currentTrajectory.sample(m_trajectoryTimer.get());

          m_desiredArmPoseDegs = state.armPositionDegs();
          m_desiredArmVelocity = state.armVelocityDegsPerSec();

          m_desiredWristPoseDegs = state.wristPositionDegs();
          m_desiredWristVelocity = state.wristVelocityDegsPerSec();
        } else {
          m_trajectoryTimer.stop();
          m_trajectoryTimer.reset();
          m_desiredState = ArmState.STOW;
          handleState();
        }
      }
      case TRAJECTORY_REVERSE -> {
        if (m_currentTrajectory != null) {
          if (m_reverseTimer == 0.0) {
            // restart the timer and find how long the last trajectory ran
            m_trajectoryTimer.stop();
            m_reverseTimer = Math.min(m_trajectoryTimer.get(), m_currentTrajectory.getFinalTime());
            m_trajectoryTimer.restart();
          } else {
            // reverse the time by getting the difference between the trajectories length and the timer
            double time = m_reverseTimer - m_trajectoryTimer.get();

            if (time <= 0.0) {
              m_desiredState = ArmState.STOW;
              handleState();
            }

            // get the current state in the trajectory
            ArmTrajectory.ArmTrajectoryState state =
                    m_currentTrajectory.sample(time);

            m_desiredArmPoseDegs = state.armPositionDegs();
            m_desiredArmVelocity = state.armVelocityDegsPerSec();

            m_desiredWristPoseDegs = state.wristPositionDegs();
            m_desiredWristVelocity = state.wristVelocityDegsPerSec();
          }
        }
      }
      case AIMBOT -> {
        // get aimbot calculations for wrist angle and use it
        ArmPose aimbotPose = AimbotUtils.aimbotCalculate(m_armPoseDegs);

        m_desiredArmPoseDegs = aimbotPose.armAngle();
        m_desiredWristPoseDegs = aimbotPose.wristAngle();
      }
      default -> {
        // do nothing
      }
    }
  }

  public void setJointAngles() {
    if (m_desiredState != ArmState.TRAJECTORY) {
      // basic setpoints
      m_armMaster.setControl(m_mm
          .withPosition(Units.degreesToRotations(m_desiredArmPoseDegs))
          .withVelocity(Units.degreesToRotations(ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue()))
          .withAcceleration(m_mm.Velocity * ArmConstants.MAX_ACCEL_S.getValue()));

      m_wristMaster.setControl(m_mm
          .withPosition(Units.degreesToRotations(m_desiredWristPoseDegs))
          .withVelocity(Units.degreesToRotations(ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue()))
          .withAcceleration(m_mm.Velocity * ArmConstants.MAX_ACCEL_S.getValue()));
    } else {
      // following a trajectory
      m_armMaster.setControl(
          m_pid.withPosition(Units.degreesToRotations(m_desiredArmPoseDegs))
              .withVelocity(Units.degreesToRotations(m_desiredArmVelocity)));

      m_wristMaster.setControl(
          m_pid.withPosition(Units.degreesToRotations(m_desiredWristPoseDegs))
              .withVelocity(Units.degreesToRotations(m_desiredWristVelocity)));
    }
  }

  public boolean atSetpoint() {
    return m_armMaster.getPosition().getValueAsDouble() == m_desiredArmPoseDegs
        && m_wristMaster.getPosition().getValueAsDouble() == m_desiredWristPoseDegs;
  }

  // We have to nudge our "zero" value because of the gear ratio
  public void resetPosition() {
    m_armMaster.setPosition(
        (m_armEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(ArmConstants.OFFSET_NUDGE))
            / ArmConstants.ARM_CANCODER_MECHANISM_RATIO);
    m_armFollower.setPosition(m_armMaster.getPosition().getValueAsDouble());

    m_wristMaster.setPosition(
        (m_wristEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(ArmConstants.OFFSET_NUDGE))
            / ArmConstants.WRIST_CANCODER_MECHANISM_RATIO);
    m_wristFollower.setPosition(m_wristMaster.getPosition().getValueAsDouble());

    if (Utils.isSimulation()) {
      m_armMaster.setPosition(Units.degreesToRotations(0.0));
      m_armFollower.setPosition(Units.degreesToRotations(0.0));

      m_wristMaster.setPosition(Units.degreesToRotations(45.0));
      m_wristFollower.setPosition(Units.degreesToRotations(45.0));
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

  // command factories

  // default command to stow
  public Command stowFactory() {
    return runOnce(() -> m_desiredState = ArmState.STOW);
  }

  public Command defaultCommandFactory() {
    return stowFactory().unless(() -> m_desiredState == ArmState.DISABLED || m_desiredState == ArmState.TRAJECTORY_REVERSE);
  }

  // set the arm to a static setpoint, use motion magic to get there
  public Command setArmSetpoint(ArmPose setpoint) {
    return run(() -> {
      m_desiredState = ArmState.SETPOINT;
      m_desiredArmPoseDegs = setpoint.armAngle();
      m_desiredWristPoseDegs = setpoint.wristAngle();
    });
  }

  public Command setArmTrajectory(ArmTrajectory traj) {
    return runEnd(() -> {
              m_desiredState = ArmState.TRAJECTORY;
              m_currentTrajectory = traj;
            },
            () -> m_desiredState = ArmState.TRAJECTORY_REVERSE);
  }

  public Command setArmState(ArmState state) {
    return runOnce(() -> m_desiredState = state);
  }

  public Command enableBrakeModeFactory(boolean enabled) {
    return runOnce(() -> m_disabledBrakeMode = enabled).ignoringDisable(true);
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(m_armSimState.getMotorVoltage());
    m_wristSim.setInputVoltage(m_wristSimState.getMotorVoltage());

    m_armSim.update(0.020);
    m_wristSim.update(0.020);

    m_armSimState.setRawRotorPosition(Units.radiansToRotations(m_armSim.getAngleRads()) * ArmConstants.ARM_SENSOR_MECHANISM_RATIO);
    m_wristSimState.setRawRotorPosition(Units.radiansToRotations(m_wristSim.getAngleRads()) * ArmConstants.WRIST_SENSOR_MECHANISM_RATIO);
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

//    BaseStatusSignal.setUpdateFrequencyForAll(
//            100,
//            m_armMaster.getPosition(),
//            m_wristMaster.getPosition()
//    );

    resetPosition();
  }
}

