// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrive;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.vision.AprilTagCameraInterface;
import frc.robot.subsystems.vision.PhotonAprilTagCamera;
import lib.utils.AimbotUtils;
import monologue.Logged;

import java.util.Optional;


public class RobotContainer implements Logged {
  private double m_maxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double m_maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // subsystems
  private final CommandSwerveDrive m_drive = TunerConstants.DriveTrain;
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // cameras
  private final AprilTagCameraInterface[] m_cameras = new AprilTagCameraInterface[] {
          new PhotonAprilTagCamera(CameraConstants.LEFT_CAMERA_NAME, CameraConstants.LEFT_CAMERA_TRANSFORMATION),
          new PhotonAprilTagCamera(CameraConstants.RIGHT_CAMERA_NAME, CameraConstants.RIGHT_CAMERA_TRANSFORMATION),
  };

  // driver controllers
  private final CommandXboxController m_driveController = new CommandXboxController(0);

  // requests for swerve drive
  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  Telemetry swerveTelemetry = new Telemetry();

  AutoFactory autoFactory = new AutoFactory();


  public RobotContainer() {
    AimbotUtils.setPoseSupplier(() -> m_drive.getState().Pose);
    m_drive.registerTelemetry(swerveTelemetry::telemeterize);

    autoFactory.setNamedCommands(m_shooter, m_arm, m_drive);

    configureBindings();
  }


  private void configureBindings() {
    // Robot default drive
    m_drive.setDefaultCommand(
            m_drive.applyRequest(() -> m_driveRequest.withVelocityX(-m_driveController.getLeftY() * m_maxSpeed)
                    .withVelocityY(-m_driveController.getLeftX() * m_maxSpeed)
                    .withRotationalRate(-m_driveController.getRightX() * m_maxAngularRate)
            ).ignoringDisable(true));

    m_arm.setDefaultCommand(m_arm.defaultCommandFactory());

    m_driveController.a().whileTrue(m_drive.applyRequest(SwerveRequest.SwerveDriveBrake::new));

        m_driveController.b().whileTrue(
            m_shooter.setShooterVelocities(3500, 3500)
        );
    }
    
    
    public Command getAutonomousCommand() {
        return autoFactory.getSelectedAutonomous();
    }

    public void updateCameras() {
      for (AprilTagCameraInterface camera : m_cameras) {
        Optional<AprilTagCameraInterface.AprilTagResult> result = camera.getLatestResult(m_drive.getState().Pose);
        result.ifPresent(m_drive::addVisionMeasurement);
      }
    }
}
