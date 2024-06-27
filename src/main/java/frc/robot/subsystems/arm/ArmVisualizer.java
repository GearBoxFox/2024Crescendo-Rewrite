package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import lib.utils.AimbotUtils;
import monologue.Logged;

public class ArmVisualizer implements Logged {
  private final String key;

  Pose3d pivotArm = new Pose3d();
  Pose3d pivotWrist = new Pose3d();

  public ArmVisualizer(String key) {
    this.key = key;
  }

  /** Update arm visualizer with current arm angle */
  public void update(double armAngleDegs, double wristAngleDegs) {
    // Log 3D poses
    pivotArm =
            new Pose3d(ArmConstants.PIVOT_JOINT_TRANSLATION.getX(), 0.0,
                    ArmConstants.PIVOT_JOINT_TRANSLATION.getY(),
                    new Rotation3d(0.0, Units.degreesToRadians(armAngleDegs), 0.0));

    pivotWrist = new Pose3d(AimbotUtils.getShooterTransformation(armAngleDegs).getTranslation(),
            new Rotation3d(0.0, Units.degreesToRadians(-wristAngleDegs), 0.0));

    log("Arm/" + key + "3d", new Pose3d[]{pivotArm, pivotWrist});
  }
}
