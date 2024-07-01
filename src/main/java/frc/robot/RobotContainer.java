// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.PointAtPoseRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmSimWrapper;
import frc.robot.subsystems.swerve.CommandSwerveDrive;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.Telemetry;
import lib.utils.AimbotUtils;
import lib.utils.FieldConstants;
import monologue.Logged;


public class RobotContainer implements Logged {
  private double m_maxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double m_maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // subsystems
  private final CommandSwerveDrive m_drive = TunerConstants.DriveTrain;
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // driver controllers
  private final CommandXboxController m_driveController = new CommandXboxController(0);

  // requests for swerve drive
  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final PointAtPoseRequest m_aimbotRequest = new PointAtPoseRequest()
          .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1)
          .withTargetPose(FieldConstants.Speaker.CENTER_SPEAKER_OPENING);
  Telemetry swerveTelemetry = new Telemetry();


  public RobotContainer() {
    AimbotUtils.setPoseSupplier(() -> m_drive.getState().Pose);
    m_drive.registerTelemetry(swerveTelemetry::telemeterize);

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
    m_driveController.x().whileTrue(m_drive.applyRequest(() -> m_aimbotRequest
            .withVelocityX(-m_driveController.getLeftY() * m_maxSpeed)
            .withVelocityY(-m_driveController.getLeftX() * m_maxSpeed)));

    m_driveController.b().whileTrue(m_arm.stowFactory());
    m_driveController.y().whileTrue(m_arm.setArmSetpoint(Constants.ArmSetpoints.INTAKE_SETPOINT));
    m_driveController.povUp().whileTrue(m_arm.setArmSetpoint(Constants.ArmSetpoints.AMP_SETPOINT));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
