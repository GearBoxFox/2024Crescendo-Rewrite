package frc.robot;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmPose;
import lib.utils.ArmTrajectory;

public class Constants {
  public static final String CANBUS_NAME = "canivore";

  public static class ArmConstants {
    private ArmConstants() {}

    public static final int WRIST_MASTER_ID = 21;
    public static final int WRIST_FOLLOWER_ID = 22;
    public static final int WRIST_ENCODER_ID = 23;
    public static final int ARM_MASTER_ID = 19;
    public static final int ARM_FOLLOWER_ID = 18;
    public static final int ARM_ENCODER_ID = 20;

    /* Because the absolute encoders are on a 2/1 ratio, we have to move our offset down a little into a rotation lower
     * than the lowest point the arm and wrist will move to , and then compensate for that in our encoder reset code */
    public static final double OFFSET_NUDGE = 45;
    public static final double ARM_OFFSET = -0.318115 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double WRIST_OFFSET = 0.000000 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double ARM_SENSOR_MECHANISM_RATIO =
        (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (64.0 / 24.0);
    public static final double ARM_CANCODER_MECHANISM_RATIO = (26.0 / 36.0) * (64.0 / 24.0);

    // the pulley on the encoder is a 1:1
    public static final double WRIST_SENSOR_MECHANISM_RATIO =
        (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (48.0 / 24.0);
    public static final double WRIST_CANCODER_MECHANISM_RATIO = (48.0 / 24.0);

    public static final double WRIST_KP = 350.0;
    public static final double WRIST_KI = 6.0;
    public static final double WRIST_KD = 3.0;
    public static final double WRIST_KS = 0.375;
    public static final double WRIST_KV = 0.0;
    public static final double WRIST_KG = 0.35;

    public static final double ARM_KP = 350.0;
    public static final double ARM_KI = 6.0;
    public static final double ARM_KD = 3.0;
    public static final double ARM_KS = 0.375;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.375;

    public static final double WRIST_LOWER_LIMIT = 0;
    public static final double WRIST_UPPER_LIMIT = 180;

    public static final double ARM_LOWER_LIMIT = -9;
    public static final double ARM_UPPER_LIMIT = 180;

    public static final double WRIST_ARM_GAP = 10;

    public static final GosDoubleProperty ARM_MAX_VELOCITY_DEG_S =
        new GosDoubleProperty(false, "Arm/Arm Max Velocity", 360);

    public static final GosDoubleProperty WRIST_MAX_VELOCITY_DEG_S =
        new GosDoubleProperty(false, "Arm/Wrist Max Velocity", 360);

    public static final GosDoubleProperty MAX_ACCEL_S =
        new GosDoubleProperty(false, "Arm/Max Accel Seconds", 0.125);

    public static final Translation2d PIVOT_JOINT_TRANSLATION =
        new Translation2d(Units.inchesToMeters(11.27),
            Units.inchesToMeters(12.56));

    public static final Transform3d PIVOT_TRANSLATION_METERS =
        new Transform3d(Units.inchesToMeters(11.27),
            0.0,
            Units.inchesToMeters(12.56),
            new Rotation3d());

    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(22.01);
    public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(14.5);
  }

  public static class ShooterConstants {
    public static final GosDoubleProperty ACCEL_COMP_FACTOR =
        new GosDoubleProperty(false, "Shooter/Acceleration Compensation", 0.100);
    public static final int TOF_ID = 28;

    private ShooterConstants() {
      throw new IllegalStateException("Static classes should not be constructed");
    }


    public static final int LEFT_ID = 25;
    public static final int RIGHT_ID = 26;
    public static final int KICKER_ID = 24;
    public static final int INTAKE_ID = 14;
    public static final int INDEXER_ID = 15;

    public static final double SHOOTER_KP = 0.110;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.00675;
    public static final double SHOOTER_KF = 0.000172;
    public static final double SHOOTER_KS = 0.21963;
    public static final double SHOOTER_KV = 0.174541;

    public static final boolean TOP_LEFT_INVERTED = false;
    public static final boolean TOP_RIGHT_INVERTED = true;
    public static final boolean BOTTOM_LEFT_INVERTED = true;
    public static final boolean BOTTOM_RIGHT_INVERTED = false;
    public static final boolean KICKER_INVERTED = true;
  }

  public static class ArmSetpoints {
    public static final ArmPose PASS_SETPOINT = new ArmPose("ArmPoses/Pass Setpoint", false, 45, 55);
    public static final ArmPose TRAP_PREPARE = new ArmPose(92.0, 145.0);
    public static final ArmPose TRAP_SCORE = new ArmPose(47.0, 120.5);

    private ArmSetpoints() {
      throw new IllegalStateException("Static classes should not be constructed");
    }

    public static final ArmPose AMP_INTERMEDIATE = new ArmPose("ArmPoses/Amp Intermediate", false, 60.0, 145.0);

    public static final ArmPose STOW_SETPOINT = new
        ArmPose("ArmPoses/Stow", true, 0.0, 35.0);
    public static final ArmPose INTAKE_SETPOINT =
        new ArmPose("ArmPoses/Intake", true, -5.75, 45.0);
    public static final ArmPose AMP_SETPOINT =
        new ArmPose("ArmPoses/Amp", true, 94.0, 145.0);

    public static final ArmPose STATIC_SHOOTER = new ArmPose("ArmPoses/ShooterTesting", false, 0.0, 55.0);

    private static final ArmTrajectory.ArmTrajectoryState start = new ArmTrajectory.ArmTrajectoryState(
            STOW_SETPOINT.wristAngle(), 0.0, STOW_SETPOINT.armAngle(), 0.0
    );

    private static final ArmTrajectory.ArmTrajectoryState middle = new ArmTrajectory.ArmTrajectoryState(
            AMP_INTERMEDIATE.wristAngle(), 0.0, AMP_INTERMEDIATE.armAngle(), 100.0
    );

    private static final ArmTrajectory.ArmTrajectoryState end = new ArmTrajectory.ArmTrajectoryState(
            AMP_SETPOINT.wristAngle(), 0.0, AMP_SETPOINT.armAngle(), 0.0
    );

    public static final ArmTrajectory AMP_TRAJECTORY = ArmTrajectory.fromCoeffs(
            ArmTrajectory.cubic_interpolation(
                    0.0, 0.5, start, middle
            ),
            0.0,
            0.5
    ).append(ArmTrajectory.fromCoeffs(
            ArmTrajectory.cubic_interpolation(
                    0.0, 0.5, middle, end
            ),
            0.0,
            0.5
    ));
  }
}
