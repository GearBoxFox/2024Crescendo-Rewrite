package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class SwerveModuleCTRE {
  private final int moduleId;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;

  private final PositionVoltage steerControl = new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage driveControl = new VelocityVoltage(0.0).withEnableFOC(true);

  private SwerveModuleState prevState;

  // status signals
  private final StatusSignal<Double> steerPositionSignal;
  private final StatusSignal<Double> steerAbsolutePositionSignal;
  private final StatusSignal<Double> drivePositionSignal;
  private final StatusSignal<Double> driveVelocitySignal;

  public SwerveModuleCTRE(
      int moduleId,
      int driveMotorId,
      int steerMotorId,
      int encoderId,
      double encoderOffsetDegrees
  ) {
    this.moduleId = moduleId;

    driveMotor = new TalonFX(driveMotorId);
    steerMotor = new TalonFX(steerMotorId);
    encoder = new CANcoder(encoderId);

    configureDriveMotor();
    configureSteerMotor();

    // setup status signals
    steerPositionSignal = steerMotor.getPosition();
    steerAbsolutePositionSignal = encoder.getPosition();
    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        steerPositionSignal,
        steerAbsolutePositionSignal,
        drivePositionSignal,
        driveVelocitySignal
    );

    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  public void setModuleState(SwerveModuleState desiredState) {
    // set angle
    if (desiredState.speedMetersPerSecond > 0.1) {
      steerMotor.setControl(steerControl.withPosition(desiredState.angle.getRotations() - steerAbsolutePositionSignal.getValueAsDouble()));
    }

    // set speed
    double rps = (desiredState.speedMetersPerSecond / TunerConstants.kWheelCircumferenceMeters) * TunerConstants.kDriveGearRatio;
    driveMotor.setControl(driveControl.withVelocity(rps));
  }

  public int getModuleId() {
    return moduleId;
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        driveVelocitySignal.getValueAsDouble() * TunerConstants.kWheelCircumferenceMeters,
        Rotation2d.fromRotations(steerPositionSignal.getValueAsDouble())
    );
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        drivePositionSignal.getValueAsDouble() * TunerConstants.kWheelCircumferenceMeters,
        Rotation2d.fromRotations(steerPositionSignal.getValueAsDouble())
    );
  }

  public void configureDriveMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentThreshold = 40;
    config.CurrentLimits.SupplyTimeThreshold = 0.1; // s

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    // module specific
    config.Slot0 = TunerConstants.driveGains;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveMotor.getConfigurator().apply(config);
  }

  public void configureSteerMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentThreshold = 30;
    config.CurrentLimits.SupplyTimeThreshold = 0.1; // s

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;

    // module specific
    config.Slot0 = TunerConstants.driveGains;

    config.Feedback.SensorToMechanismRatio = TunerConstants.kSteerGearRatio;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    steerMotor.getConfigurator().apply(config);
  }

  public void configureEncoder(double offset) {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    config.MagnetSensor.MagnetOffset = Units.degreesToRotations(offset);

    encoder.getConfigurator().apply(config);
  }
}
