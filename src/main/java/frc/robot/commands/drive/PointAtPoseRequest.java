package frc.robot.commands.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class PointAtPoseRequest implements SwerveRequest {
  // The steer request type
  private SwerveModule.SteerRequestType m_steerRequestType = SwerveModule.SteerRequestType.MotionMagic;

  // the drive request type
  private SwerveModule.DriveRequestType m_driveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    return null;
  }
}
