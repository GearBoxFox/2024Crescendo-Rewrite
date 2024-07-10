package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import lib.utils.FieldConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonAprilTagCamera implements AprilTagCameraInterface {
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;

  private final double xyStdDevCoefficient = Units.inchesToMeters(4.0);
  private final double thetaStdDevCoefficient = Units.degreesToRadians(7.5);

  private final double xyStdDevMultiTagCoefficient = Units.inchesToMeters(2.5);
  private final double thetaStdDevMultiTagCoefficient = Units.degreesToRadians(2.0);

  // simulation
  private VisionSystemSim m_visSim;
  private PhotonCameraSim m_camSim;

  public PhotonAprilTagCamera(String name, Transform3d transform) {
    m_camera = new PhotonCamera(name);
    m_poseEstimator = new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            transform
    );

    m_poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      m_camSim = new PhotonCameraSim(m_camera);
      m_visSim = new VisionSystemSim(m_camera.getName());
      m_visSim.addAprilTags(m_poseEstimator.getFieldTags());
    }
  }

  @Override
  public Optional<AprilTagResult> getLatestResult(Pose2d referencePose) {
    // get the latest result
    m_poseEstimator.setReferencePose(referencePose);
    PhotonPipelineResult result = m_camera.getLatestResult();
    Optional<EstimatedRobotPose> opPose = m_poseEstimator.update(result);

    // update sim
    if (RobotBase.isSimulation()) {
      m_visSim.update(referencePose);
      m_camSim.submitProcessedFrame(result);
    }

    return opPose.map(estimatedRobotPose -> new AprilTagResult(
            estimatedRobotPose.estimatedPose.toPose2d(),
            getStandardDeviations(estimatedRobotPose)
    ));
  }

  public Matrix<N3, N1> getStandardDeviations(EstimatedRobotPose result) {
    if (result.targetsUsed.isEmpty()) {
      return VecBuilder.fill(0.0, 0.0, 0.0);
    }

    // get average distance to all visible tags
    int numTags = 0;
    double avgDistance = 0;

    for (var target : result.targetsUsed) {
      var tagPose = m_poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      // check for non-valid tag ids
      if (tagPose.isEmpty()) {
        continue;
      }

      numTags++;
      avgDistance += tagPose.get()
              .toPose2d()
              .getTranslation()
              .getDistance(result.estimatedPose.toPose2d().getTranslation());
    }

    avgDistance /= numTags;

    double xyStdDev;
    double thetaStdDev;

    if (numTags > 1.0) {
      xyStdDev = xyStdDevMultiTagCoefficient * Math.pow(avgDistance, 2.0);
      thetaStdDev = thetaStdDevMultiTagCoefficient * Math.pow(avgDistance, 2.0);
    } else {
      xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0);
      thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0);
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }
}
