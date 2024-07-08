package lib.utils;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmPose;
import monologue.Annotations;

import java.util.function.Supplier;

public class AimbotUtils {
  private static final GosDoubleProperty m_offsetInches =
      new GosDoubleProperty(false, "Wrist Angle Offset In", 0);
  private static final GosDoubleProperty m_offsetDegrees =
      new GosDoubleProperty(false, "Wrist Angle Degs offset", 1.6);

  private static Supplier<Pose2d> m_poseSupplier;

  private static final double Y_TARGET = 0.2;

  public static void setPoseSupplier(Supplier<Pose2d> pose2dSupplier) {
    m_poseSupplier = pose2dSupplier;
  }

  /** Linear interpolation tables for aiming */
  public static double getWristAngle(double dist) {
    double distance = dist - m_offsetInches.getValue();
    return 49.15 - 0.123 * distance
        + 0.0015 * Math.pow((distance - 117.647), 2)
        - 1.628e-5 * Math.pow((distance - 117.647), 3)
        + m_offsetDegrees.getValue();
  }

  @Annotations.Log(key = "Left Shooter Setpoint")
  public static double getLeftSpeed() {
    double distance = getDistanceFromSpeaker(m_poseSupplier.get());
    if (75.0 > distance) {
      return 4500;
    } else if (distance > 170.0) {
      return 5500;
    } else {
      return 4750;
    }
  }

  @Annotations.Log(key = "Right Shooter Setpoint")
  public static double getRightSpeed() {
    double distance = getDistanceFromSpeaker(m_poseSupplier.get());
    if (75.0 > distance) {
      return 3000;
    } else if (distance > 170.0) {
      return 4000;
    } else {
      return 3250;
    }
  }

  /** Gets the distance from the drivebase to the speaker in meters */
  public static double getDistanceFromSpeaker(Pose2d drivePose) {
    return AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER).toTranslation2d()
        .getDistance(drivePose.getTranslation());
  }

  /** Gets the angle the drivebase should be at with a default of the speaker */
  public static Rotation2d getDrivebaseAimingAngle(Pose2d drivePose) {
    return getDrivebaseAimingAngle(drivePose, FieldConstants.CENTER_SPEAKER);
  }

  /** Gets the angle the drivebase should be at to aim at the speaker */
  public static Rotation2d getDrivebaseAimingAngle(Pose2d drivePose, Translation3d target) {
    Transform3d robotToPoint =
        new Transform3d(
            new Pose3d(new Pose2d(drivePose.getTranslation(), new Rotation2d())),
            new Pose3d(AllianceFlipUtil.apply(target), new Rotation3d()));

    return Rotation2d.fromRadians(
        MathUtil.angleModulus(
            Math.PI * 2 - (Math.atan2(robotToPoint.getX(), robotToPoint.getY()))
                + Units.degreesToRadians(90)
        ));
  }

  /** Gets the transformation of the shooter relative to the drive base */
  public static Transform3d getShooterTransformation(double armAngle) {
    return ArmConstants.PIVOT_TRANSLATION_METERS.plus(
        // Add the movement of the arm
        GeomUtils.translationToTransform(new Translation3d(
            ArmConstants.ARM_LENGTH_METERS,
            new Rotation3d(0.0, Units.degreesToRadians(armAngle + 180), 0.0)
        ))
    );
  }

  public static ArmPose aimbotCalculate(double armAngle) {
    Pose3d robotPose = new Pose3d(m_poseSupplier.get());

    // the position to target
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    Translation2d speakerPoseGround = speakerPose.getTranslation().toTranslation2d();

    // find where the shooter is at relative to the robot
    Pose3d shooterPivotPose = robotPose.plus(getShooterTransformation(armAngle));
    // find the transformation from the shooter to the speaker
    Transform3d robotToSpeaker =
        new Transform3d(shooterPivotPose, speakerPose);

    // find the distance on the ground to the speaker
    double groundDistance = robotPose.getTranslation().toTranslation2d().getDistance(speakerPoseGround);

    double desiredWristAngle = Units.radiansToDegrees(Math.atan(robotToSpeaker.getZ()/groundDistance));
    desiredWristAngle = desiredWristAngle + (90 - desiredWristAngle) * 0.06;

    double safeArmAngle = ArmConstants.WRIST_ARM_GAP - desiredWristAngle;
    safeArmAngle = safeArmAngle >= 0 ? safeArmAngle : 0;

    return new ArmPose(safeArmAngle, desiredWristAngle);
  }
}