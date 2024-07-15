package frc.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrive;
import lib.utils.AimbotUtils;
import monologue.Logged;


public class ShooterAutoCommand extends Command implements Logged {
  private final double m_maxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final double m_maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final ArmSubsystem m_armSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final CommandSwerveDrive m_driveSubsystem;

  private final Timer m_timer;

  private boolean m_previousHadPiece;
  private boolean m_hasChanged;

  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public ShooterAutoCommand(ArmSubsystem armSubsystem,
                            ShooterSubsystem shooterSubsystem,
                            CommandSwerveDrive driveSubsystem) {
    this.m_armSubsystem = armSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_driveSubsystem = driveSubsystem;

    m_timer = new Timer();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_armSubsystem, this.m_shooterSubsystem, driveSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_previousHadPiece = m_shooterSubsystem.hasGamePiece();

    m_hasChanged = false;
  }

  @Override
  public void execute() {
    // Calculate output to align to speaker
    Rotation2d desiredAngle = AimbotUtils.getDrivebaseAimingAngle();
    double omega = m_driveSubsystem.alignToAngle(desiredAngle.getDegrees());
    m_driveSubsystem.setControl(m_driveRequest.withRotationalRate(omega / 360.0));

    double error = Math.abs(desiredAngle.getDegrees() - m_driveSubsystem.getState().Pose.getRotation().getDegrees());
    log("Shooter Auto Error", error);

    log("ShooterAtSpeed?", m_shooterSubsystem.atSetpoint());
    log("DriveBaseGood?", error < 10.0);
    log("WristAtSetpoint", m_armSubsystem.wristAtSetpoint());

    // only actually shoot if we're aligned close enough to speaker and flywheels are at speed
    m_shooterSubsystem.runVelocity(m_shooterSubsystem.atSetpoint()
            && error < 10.0 && m_armSubsystem.wristAtSetpoint()).execute();
    m_armSubsystem.setArmState(ArmSubsystem.ArmState.AIMBOT);

    // check to see if the state of having a note has changed, mark if it has
    if (m_previousHadPiece != m_shooterSubsystem.hasGamePiece()) {
      m_hasChanged = true;
    }

    m_previousHadPiece = m_shooterSubsystem.hasGamePiece();
  }

  @Override
  public boolean isFinished() {
    // if magazine is empty, we've detected a note has run through the system, and we're not taking too
    // short or too long to run

    // too long check may change as system gets tuned
    return (!m_shooterSubsystem.hasGamePiece() && m_hasChanged
            && m_timer.hasElapsed(0.75)) || m_timer.hasElapsed(5.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterPowers(0.0, 0.0);
    m_shooterSubsystem.setKickerPower(0.0);
    m_armSubsystem.setArmState(ArmSubsystem.ArmState.STOW);
  }

}