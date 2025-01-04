package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.module.SwerveModuleCTRE;
import monologue.Annotations;

import java.awt.*;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModuleCTRE[] modules;
  private final SwerveDriveKinematics kinematics;
  private final Pigeon2 gyro;
  private final SwerveDriveOdometry odometry;

  public SwerveDriveSubsystem() {
    modules = new SwerveModuleCTRE[] {
        new SwerveModuleCTRE(
            0,
            TunerConstants.kFrontLeftDriveMotorId,
            TunerConstants.kFrontLeftSteerMotorId,
            TunerConstants.kFrontLeftEncoderId,
            TunerConstants.kFrontLeftEncoderOffset * 360),
        new SwerveModuleCTRE(
            1,
            TunerConstants.kFrontRightDriveMotorId,
            TunerConstants.kFrontRightSteerMotorId,
            TunerConstants.kFrontRightEncoderId,
            TunerConstants.kFrontRightEncoderOffset * 360),
        new SwerveModuleCTRE(
            2,
            TunerConstants.kBackLeftDriveMotorId,
            TunerConstants.kBackLeftSteerMotorId,
            TunerConstants.kBackLeftEncoderId,
            TunerConstants.kBackLeftEncoderOffset * 360),
        new SwerveModuleCTRE(
            3,
            TunerConstants.kBackRightDriveMotorId,
            TunerConstants.kBackRightSteerMotorId,
            TunerConstants.kBackRightEncoderId,
            TunerConstants.kBackRightEncoderOffset * 360)
    };

    gyro = new Pigeon2(TunerConstants.kPigeonId);

    kinematics = new SwerveDriveKinematics(
        new Translation2d(TunerConstants.kFrontLeftXPosInches, TunerConstants.kFrontLeftYPosInches).times(0.0254),
        new Translation2d(TunerConstants.kFrontRightXPosInches, TunerConstants.kFrontRightYPosInches).times(0.0254),
        new Translation2d(TunerConstants.kBackLeftXPosInches, TunerConstants.kBackLeftYPosInches).times(0.0254),
        new Translation2d(TunerConstants.kBackRightXPosInches, TunerConstants.kBackRightYPosInches).times(0.0254)
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions(), new Pose2d());
  }

  @Override
  public void periodic() {
    // update odometry
    odometry.update(getHeading(), getModulePositions());
  }

  public void runChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    for (SwerveModuleCTRE module : modules) {
      module.setModuleState(states[module.getModuleId()]);
    }
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getModulePosition();
    }

    return positions;
  }

  @Annotations.Log()
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getModuleState();
    }

    return states;
  }

  public Command driveFactory(CommandXboxController controller) {
    return run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          controller.getLeftX() * TunerConstants.kSpeedAt12VoltsMps,
          -controller.getLeftY() * TunerConstants.kSpeedAt12VoltsMps,
          controller.getRightX()
      );

      runChassisSpeeds(speeds);
    });
  }
}

