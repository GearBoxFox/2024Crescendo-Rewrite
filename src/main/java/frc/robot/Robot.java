// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
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
        DriverStation.silenceJoystickConnectionWarning(true);
        robotContainer = new RobotContainer();

        boolean lazyLogging = false;
        boolean fileOnly = false;
        Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
    }
    
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        robotContainer.updateCameras();

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
