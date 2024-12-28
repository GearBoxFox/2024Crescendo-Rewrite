package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.module.SwerveModuleCTRE;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModuleCTRE[] modules;
  private final SwerveDriveKinematics kinematics;
  private final Pigeon2 gyro;

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
        new Translation2d(TunerConstants.kFrontLeftXPosInches, TunerConstants.kFrontLeftYPosInches),
        new Translation2d(TunerConstants.kFrontRightXPosInches, TunerConstants.kFrontRightYPosInches),
        new Translation2d(TunerConstants.kBackLeftXPosInches, TunerConstants.kBackLeftYPosInches),
        new Translation2d(TunerConstants.kBackRightXPosInches, TunerConstants.kBackRightYPosInches)
    );
  }

  public void runChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    for (SwerveModuleCTRE module : modules) {
      module.setModuleState(states[module.getModuleId()]);
    }
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

