package lib.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmPose;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Cubic spline trajectories
public class ArmTrajectory {
  private double[] m_times;
  private ArmTrajectoryState[] m_states;
  private double m_startTime;
  private double m_finalTime;

  public record ArmTrajectoryState(
      double wristPositionDegs,
      double wristVelocityDegsPerSec,
      double armPositionDegs,
      double armVelocityDegsPerSec
  ) {}

  public ArmTrajectory(double[] times, ArmTrajectoryState[] states) {
    if (times.length != states.length) {
      throw new IllegalStateException("Number of states must match number of times");
    }

    m_times = times;
    m_states = states;
    m_startTime = m_times[0];
    m_finalTime = m_times[m_times.length - 1];
  }

  public double getFinalTime() {
    return m_finalTime;
  }

  public double clipTime(double time) {
    /*
    Limits a given time between the trajectories start and end time
    */

    return MathUtil.clamp(time, m_startTime, m_finalTime);
  }

  public ArmTrajectory append(ArmTrajectory other) {
    double[] otherTimes =
        Arrays.stream(other.m_times)
            .map((double time) -> time + m_finalTime)
            .toArray();

    double[] newTimes = new double[m_times.length + otherTimes.length];

    // combine timestamps for both trajectories
    for (int i = 0; i < m_times.length + otherTimes.length; i++) {
      if (i >= m_times.length) {
        newTimes[i] = otherTimes[i - m_times.length];
      } else {
        newTimes[i] = m_times[i];
      }
    }

    ArmTrajectoryState[] newStates =
        new ArmTrajectoryState[m_states.length + other.m_states.length];

    // combine states for both trajectories
    for (int i = 0; i < m_states.length + other.m_states.length; i++) {
      if (i >= m_states.length) {
        newStates[i] = other.m_states[i - m_states.length];
      } else {
        newStates[i] = m_states[i];
      }
    }

    return new ArmTrajectory(newTimes, newStates);
  }

  public ArmTrajectoryState sample(double desiredTime) {
    /*
    Samples the trajectory at a given time.

    Linearly interpolates between trajectory samples.
    */

    // finding the indexes of the next and last closest times
    double time = clipTime(desiredTime);
    int nextIdx = nextTimeIdx(m_times, time);
    int prevIdx = prevTimeIdx(m_times, time);

    if (prevIdx == nextIdx) {
      return m_states[prevIdx];
    }

    // find the next and last closest states and times
    ArmTrajectoryState prevState = m_states[prevIdx];
    ArmTrajectoryState nextState = m_states[nextIdx];
    double prevTime = m_times[prevIdx];
    double nextTime = m_times[nextIdx];

    // lots of math to create lots of terms ;w;
    double newArmPosition =
        prevState.armPositionDegs + (nextState.armPositionDegs - prevState.armPositionDegs)
            / (nextTime - prevTime) * (time - prevTime);

    double newArmVelocity =
        prevState.armVelocityDegsPerSec + (nextState.armVelocityDegsPerSec - prevState.armVelocityDegsPerSec)
            / (nextTime - prevTime) * (time - prevTime);

    double newWristPosition =
        prevState.wristPositionDegs + (nextState.wristPositionDegs - prevState.wristPositionDegs)
            / (nextTime - prevTime) * (time - prevTime);

    double newWristVelocity =
        prevState.wristVelocityDegsPerSec + (nextState.wristVelocityDegsPerSec - prevState.wristVelocityDegsPerSec)
            / (nextTime - prevTime) * (time - prevTime);

    return new ArmTrajectoryState(newWristPosition, newWristVelocity, newArmPosition, newArmVelocity);
  }

  public static ArmTrajectory generate(ArmTrajectoryState... poses) {
    /*
    * Generate a trajectory given a list of arm poses for each setpoint.
    *
    * Assumes max velocity for each joint in intermediate setpoints unless the joint location is the same
    */

    ArmTrajectory traj = null;

    double armMaxAccel = ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue() / ArmConstants.MAX_ACCEL_S.getValue();
    double wristMaxAccel = ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue() / ArmConstants.MAX_ACCEL_S.getValue();

    for (int i = 0; i < poses.length - 1; i++) {
      ArmTrajectoryState currentPose = poses[i];
      ArmTrajectoryState nextPose = poses[i + 1];

      double segmentTime = calcWaypointTime(currentPose, nextPose, armMaxAccel, wristMaxAccel);

      DriverStation.reportWarning("Current Angle: " + currentPose.armPositionDegs +
          "Desired Angle: " + nextPose.armPositionDegs +
          "Max Acceleration: " + armMaxAccel +
          "Delta Time: " + segmentTime, false);

      SmartDashboard.putNumber("Delta Time: ", segmentTime);

      // make sure that the trajectory isn't null
      if (traj != null) {
        traj.append(
            ArmTrajectory.fromCoeffs(
                ArmTrajectory.cubic_interpolation(0.0, segmentTime, currentPose, nextPose),
                0.0,
                segmentTime
            )
        );
      } else {
        traj = ArmTrajectory.fromCoeffs(
            ArmTrajectory.cubic_interpolation(0.0, segmentTime, currentPose, nextPose),
            0.0,
            segmentTime
        );
      }
    }

    return traj;
  }

  private static double calcWaypointTime(ArmTrajectoryState i,
                                         ArmTrajectoryState f,
                                         double armMaxAccel,
                                         double wristMaxAccel) {
    double armDelta = Math.copySign(f.armPositionDegs - i.armPositionDegs, 1.0);
    double wristDelta = Math.copySign(f.wristPositionDegs - i.wristPositionDegs, 1.0);

    double armTotalTime = 0.0;
    double wristTotalTime = 0.0;

    // find the time each join must take to travel
    //arm
    double armAccelTime = (ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue() - i.armVelocityDegsPerSec) / armMaxAccel;
    double armAccelDistance = (1.0 / 2.0) * armMaxAccel * Math.pow(armAccelTime, 2);

    double armDecelTime = (f.armVelocityDegsPerSec - ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue()) / -armMaxAccel;
    double armDecelDistance = (1.0 / 2.0) * armMaxAccel * Math.pow(armDecelTime, 2);

    // we travel too far accelerating or decelerating so we can't accel/decel to max velocity
    if ((armAccelDistance + armDecelDistance) > armDelta) {
      // figure out the max distance we can accel and decel
      // s = a/2 * t^2
      // creates an equal triangle accel/decel curve
      armTotalTime =  2 * Math.sqrt(armDelta / armMaxAccel);
      SmartDashboard.putBoolean("Arm Edgecase", true);
    } else {
      double maxVelTime = ArmConstants.ARM_MAX_VELOCITY_DEG_S.getValue() / (armDelta - armAccelDistance - armDecelDistance);
      armTotalTime =  maxVelTime + armAccelTime + armDecelTime;
      SmartDashboard.putBoolean("Arm Edgecase", false);
    }

    // wrist
    double wristAccelTime = (ArmConstants.WRIST_MAX_VELOCITY_DEG_S.getValue() - i.wristVelocityDegsPerSec) / wristMaxAccel;
    double wristAccelDistance = (1.0 / 2.0) * wristMaxAccel * Math.pow(wristAccelTime, 2);

    double wristDecelTime = (f.wristVelocityDegsPerSec - ArmConstants.WRIST_MAX_VELOCITY_DEG_S.getValue()) / -wristMaxAccel;
    double wristDecelDistance = (1.0 / 2.0) * wristMaxAccel * Math.pow(wristDecelTime, 2);

    // we travel too far accelerating or decelerating so we can't accel/decel to max velocity
    if ((wristAccelDistance + wristDecelDistance) > wristDelta) {
      // figure out the max distance we can accel and decel
      // s = a/2 * t^2
      // creates an equal triangle accel/decel curve
      wristTotalTime =  2 * Math.sqrt(wristDelta / wristMaxAccel);
      SmartDashboard.putBoolean("Wrist Edgecase", true);
    } else {
      double maxVelTime = ArmConstants.WRIST_MAX_VELOCITY_DEG_S.getValue() / (wristDelta - wristAccelDistance - wristDecelDistance);
      wristTotalTime =  maxVelTime + wristAccelTime + wristDecelTime;
      SmartDashboard.putBoolean("Wrist Edgecase", false);
    }

    return Math.max(armTotalTime, wristTotalTime);
  }

  public static ArmTrajectory fromCoeffs(SimpleMatrix coeffs, double t_0, double t_f) {
    /*
    Generate a trajectory from a polynomial coefficients matrix.

    Keyword arguments:
    coeffs -- Polynomial coefficients as columns in increasing order.
              Can have arbitrarily many columns.
    t_0 -- time to start the interpolation
    t_f -- time to end the interpolation

    This will only create a quadratic function, it will not create a quintic one.

    Returns:
    Trajectory following the interpolation. The states will be in the form:
    [pos_1, ..., pos_n, vel_1, ..., vel_n]
    Where n is the number of columns in coeffs.
     */

    // create an array of timestamps to sample from
    ArrayList<Double> tList = new ArrayList<>();
    double averageDelta = (t_f - t_0) / 100.0;
    double x = 0;
    for (int i = 0; i < 100; i++) {
      tList.add(x + t_0);
      x += averageDelta;
    }
    tList.add(t_f);

    double[] time = tList.stream().mapToDouble(Double::valueOf).toArray();

    double[] tPow0 = tList.stream().mapToDouble(Double::valueOf).map((double value) -> Math.pow(value, 0)).toArray();
    double[] tPow1 = tList.stream().mapToDouble(Double::valueOf).map((double value) -> Math.pow(value, 1)).toArray();
    double[] tPow2 = tList.stream().mapToDouble(Double::valueOf).map((double value) -> Math.pow(value, 2)).toArray();
    double[] tPow3 = tList.stream().mapToDouble(Double::valueOf).map((double value) -> Math.pow(value, 3)).toArray();

    SimpleMatrix posTimeMatrix = new SimpleMatrix(new double[101][4]);
    posTimeMatrix.setColumn(0, 0, tPow0);
    posTimeMatrix.setColumn(1, 0, tPow1);
    posTimeMatrix.setColumn(2, 0, tPow2);
    posTimeMatrix.setColumn(3, 0, tPow3);

    SimpleMatrix posVec = posTimeMatrix.mult(coeffs);

    SimpleMatrix multVec = repeatArange(3,100);
    SimpleMatrix partialPosT = new SimpleMatrix(posTimeMatrix.getColumn(0))
        .concatColumns(posTimeMatrix.getColumn(1))
        .concatColumns(posTimeMatrix.getColumn(2))
        .mult(multVec);

    SimpleMatrix velTVec = SimpleMatrix.filled(100, 1, 0)
        .concatColumns(partialPosT.getColumn(0))
        .concatColumns(partialPosT.getColumn(1))
        .concatColumns(partialPosT.getColumn(2));

    SimpleMatrix velVec = velTVec.mult(coeffs);

    ArmTrajectoryState[] states = new ArmTrajectoryState[101];
    double[][] poseStates = posVec.toArray2();
    double[][] velStates = velVec.toArray2();

    for (int i = 0; i < 101; i++) {
      ArmTrajectoryState temp = new ArmTrajectoryState(
          poseStates[i][1],
          velStates[i][1],
          poseStates[i][0],
          velStates[i][0]
      );

      states[i] = temp;
    }

    return new ArmTrajectory(time, states);
  }

  public static SimpleMatrix cubic_interpolation(
      double t_0,
      double t_f,
      ArmTrajectoryState state_0,
      ArmTrajectoryState state_f) {
    /*
    Perform cubic interpolation between state₀ at t = t₀ and state_f at t = t_f.

    Returns:
    coeffs -- 4x2 matrix containing the interpolation coefficients for joint 1
              in column 1 and joint 2 in column 2
    */

    // copy our states into the "output" matrix
    SimpleMatrix rhs = new SimpleMatrix(new double[4][2]);

    rhs.setColumn(0, 0,
        state_0.armPositionDegs,
        state_0.armVelocityDegsPerSec,
        state_f.armPositionDegs,
        state_f.armVelocityDegsPerSec);

    rhs.setColumn(1, 0,
        state_0.wristPositionDegs,
        state_0.wristVelocityDegsPerSec,
        state_f.wristPositionDegs,
        state_f.wristVelocityDegsPerSec);

    SimpleMatrix lhsSimple = SimpleMatrix.filled(4, 4, 0.0);
    lhsSimple.setRow(0, 0, posRow(t_0));
    lhsSimple.setRow(1, 0, velRow(t_0));
    lhsSimple.setRow(2, 0, posRow(t_f));
    lhsSimple.setRow(3, 0, velRow(t_f));


    return new Matrix<>(lhsSimple).inv().times(new Matrix<>(rhs)).getStorage();
  }

  private static double[] posRow(double t) {
    return new double[]{1, t, t * t, t * t * t};
  }

  private static double[] velRow(double t) {
    return new double[]{0, 1, 2 * t, 3 * t * t};
  }

  private static int nextTimeIdx(double[] array, double time) {
    int returnvalue = -1;
    for (int i = 0; i < array.length; ++i) {
      if (array[i] >= time) {
        returnvalue = i;
        break;
      }
    }
    return returnvalue;
  }

  private static int prevTimeIdx(double[] array, double time) {
    int returnvalue = -1;
    for (int i = 0; i < array.length; ++i) {
      if (array[i] <= time) {
        returnvalue = i;
      }
    }
    return returnvalue;
  }

  private static double[] arange(int order) {
    ArrayList<Double> tList = new ArrayList<>();
    double step = 1;
    double x = 0;
    for (int i = 0; i < order; i++) {
      tList.add(x);
      x += step;
    }

    double[] time = tList.stream().mapToDouble(Double::valueOf).map((double value) -> Math.pow(value, 0)).toArray();
    return time;
  }

  private static SimpleMatrix repeatArange(int order, int n) {
    SimpleMatrix temp = new SimpleMatrix(new double[order][n]);
    for (int i = 0; i < n; i++) {
      temp.setColumn(i, 0, arange(order));
    }

    return temp;
  }
}