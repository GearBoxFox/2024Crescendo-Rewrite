package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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

    public static final Translation2d PIVOT_JOINT_TRANSLATION =
        new Translation2d(Units.inchesToMeters(9.27),
            Units.inchesToMeters(12.56));

    public static final Transform3d PIVOT_TRANSLATION_METERS =
        new Transform3d(Units.inchesToMeters(9.27),
            0.0,
            Units.inchesToMeters(12.56),
            new Rotation3d());

    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(22.01);
    public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(14.5);
  }
}
