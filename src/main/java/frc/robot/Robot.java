// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.utils.ArmTrajectory;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;

    private ArmTrajectory temp;
    
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        boolean lazyLogging = false;
        boolean fileOnly = false;
        Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);

        double t0 = 0.0;
        double tf= 1.0;
        ArmTrajectory.ArmTrajectoryState state0 =
            new ArmTrajectory.ArmTrajectoryState(0.0, 0.0, 0.0, 0.0);
        ArmTrajectory.ArmTrajectoryState statef =
            new ArmTrajectory.ArmTrajectoryState(90.0, 90.0, 0.0, 0.0);

        temp = ArmTrajectory.fromCoeffs(
            ArmTrajectory.cubic_interpolation(
                t0,
                tf,
                state0,
                statef
            ),
            t0,
            tf
        );
    }
    
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Monologue.setFileOnly(DriverStation.isFMSAttached());
        Monologue.updateAll();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
