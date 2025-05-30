package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.AprilTagCameraInterface;
import monologue.Logged;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrive extends SwerveDrivetrain implements Subsystem, Logged {
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d m_blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d m_redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          null,
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
          null,
          this));

  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          null,
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> setControl(RotationCharacterization.withVolts(volts)),
          null,
          this));
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(7),
          null,
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> setControl(SteerCharacterization.withVolts(volts)),
          null,
          this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine m_routineToApply = m_sysIdRoutineTranslation;

  public CommandSwerveDrive(SwerveDrivetrainConstants driveTrainConstants, double odometryUpdateFrequency, SwerveModuleConstants... modules) {
    super(driveTrainConstants, odometryUpdateFrequency, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        ()->this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative,  // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_routineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_routineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public void addVisionMeasurement(AprilTagCameraInterface.AprilTagResult measurement) {
    addVisionMeasurement(
            measurement.estimatedPose(),
            measurement.timestamp(),
            measurement.standardDeviations()
    );
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? m_redAlliancePerspectiveRotation
                : m_blueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
      });
    }
  }
}