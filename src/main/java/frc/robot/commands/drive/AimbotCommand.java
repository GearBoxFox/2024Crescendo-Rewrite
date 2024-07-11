package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrive;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.AimbotUtils;
import monologue.Logged;


public class AimbotCommand extends Command implements Logged {
  private final double m_maxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final double m_maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final ArmSubsystem m_arm;
  private final CommandSwerveDrive m_drive;
  private final ShooterSubsystem m_shooter;
  private final CommandXboxController m_driveController;

  private final PIDController m_headingController;
  private final boolean m_runKicker;

  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public AimbotCommand(ArmSubsystem arm,
                       CommandSwerveDrive drive,
                       ShooterSubsystem shooter,
                       CommandXboxController driveController,
                       boolean runKicker) {
    this.m_arm = arm;
    this.m_drive = drive;
    this.m_shooter = shooter;
    this.m_driveController = driveController;
    this.m_runKicker = runKicker;

    m_headingController = new PIDController(0.1, 0.0, 0.002);
    m_headingController.setTolerance(7.5); // degrees
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_arm, this.m_drive, this.m_shooter);
  }

  @Override
  public void execute() {
    // get our drive inputs and apply deadband
    double driveX = -m_driveController.getLeftY();
    double driveY = -m_driveController.getLeftX();

    driveX = MathUtil.applyDeadband(driveX, 0.1);
    driveY = MathUtil.applyDeadband(driveY, 0.1);

    // find the heading output
    double headingOutput = m_headingController.calculate(
            m_drive.getState().Pose.getRotation().getDegrees(),
            AimbotUtils.getDrivebaseAimingAngle().getDegrees()
    );

    // set the robot to drive
    m_drive.setControl(m_driveRequest
            .withVelocityX(driveX * m_maxSpeed)
            .withVelocityY(driveY * m_maxSpeed)
            .withRotationalRate(Units.degreesToRotations(headingOutput)));

    // run shooter
    m_shooter.setShooterVelocities(
            AimbotUtils.getLeftSpeed(),
            AimbotUtils.getRightSpeed()
    ).execute();
    m_shooter.runKicker(m_runKicker);

    // set arm
    m_arm.setArmState(ArmSubsystem.ArmState.AIMBOT).execute();

    log("Shooter at speed", m_shooter.atSetpoint());
    log("Arm at setpoint", m_arm.atSetpoint());
    log("Drivebase at setpoint", m_headingController.atSetpoint());

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
