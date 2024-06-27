package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import monologue.Logged;

public class Telemetry implements Logged {

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d pose = state.Pose;
    log("Robot Pose", pose);

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    log("Robot Speed", velocities.getNorm());
    log("Velocity X", velocities.getX());
    log("Velocity Y", velocities.getY());
    log("Odometry Frequency", 1.0 / state.OdometryPeriod);

    /* Telemeterize the module's states */
    log("Module States", state.ModuleStates);
    log("Module Setpoints", state.ModuleTargets);
  }
}