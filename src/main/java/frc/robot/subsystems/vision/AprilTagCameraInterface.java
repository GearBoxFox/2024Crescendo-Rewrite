package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;

public interface AprilTagCameraInterface {
  record AprilTagResult (
          Pose2d estimatedPose,
          Matrix<N3, N1> standardDeviations
  ) {}

  Optional<AprilTagResult> getLatestResult(Pose2d referencePose);
}
