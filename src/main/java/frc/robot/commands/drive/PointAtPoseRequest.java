package frc.robot.commands.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import lib.utils.AllianceFlipUtil;

public class PointAtPoseRequest extends SwerveRequest.FieldCentricFacingAngle {
  public Pose2d m_targetPose = new Pose2d();
  public Pose2d m_robotPose = new Pose2d();

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    Transform2d robotToTarget = new Transform2d(m_robotPose,
        AllianceFlipUtil.apply(m_targetPose));


    TargetDirection = Rotation2d.fromRadians(
        MathUtil.angleModulus(
            Math.PI * 2 - (Math.atan2(robotToTarget.getX(), robotToTarget.getY()))
                + Units.degreesToRadians(90)
        ));

    return super.apply(parameters, modulesToApply);
  }

  public PointAtPoseRequest withTargetPose(Pose2d pose) {
    m_targetPose = pose;
    return this;
  }

  public PointAtPoseRequest withTargetPose(Pose3d pose) {
    m_targetPose = pose.toPose2d();
    return this;
  }

  public PointAtPoseRequest withRobotPose(Pose2d robotPose) {
    m_robotPose = robotPose;
    return this;
  }

  // Fill in the rest of the wrapper
  public PointAtPoseRequest withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s.
   * Y is defined as to the left according to WPILib convention,
   * so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  public PointAtPoseRequest withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * Sets the desired direction to face.
   * 0 Degrees is defined as in the direction of the X axis.
   * As a result, a TargetDirection of 90 degrees will point along
   * the Y axis, or to the left.
   *
   * @param targetDirection Desired direction to face
   * @return this request
   */
  public PointAtPoseRequest withTargetDirection(Rotation2d targetDirection) {
    this.TargetDirection = targetDirection;
    return this;
  }

  public PointAtPoseRequest withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  public PointAtPoseRequest withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }

  public PointAtPoseRequest withCenterOfRotation(Translation2d centerOfRotation) {
    this.CenterOfRotation = centerOfRotation;
    return this;
  }

  public PointAtPoseRequest withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }

  public PointAtPoseRequest withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }
}
