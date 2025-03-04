// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.preferences.SKPreferences;


/**
 * The VM is configured to automatically run this class, and to call the functions
 * corresponding to each mode, as described in the TimedRobot documentation. If you change
 * the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends LoggedRobot
{
    public static final double kDefaultPeriod = 0.2;

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    SendableChooser<Command> autoCommandSelector = new SendableChooser<Command>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
        // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);    //TODO: add this back, caused memory issue
        // if (isReal()) {
        //     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        // } else {
        //     setUseTiming(false); // Run as fast as possible
        //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        // }

        // Logger.start();
        //DataLogManager.start(); TODO - look at data logs to see if they work with advantage scope
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        //Schedule a warmup command to prevent delay at the begenning of auto. This helps aleviate a 
        //java specific isssue of the auto starting late.
        FollowPathCommand.warmupCommand().schedule();


        //get the saved elastic dashboard layout
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Logger.recordOutput("RobotPose", new Pose2d());
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        //display preferences periodicaly
        SKPreferences.refreshIfNeeded();

        //display real time memory consumption
        SmartDashboard.putNumber("Memory", Runtime.getRuntime().freeMemory());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer}
     * class.
     */
    @Override
    public void autonomousInit()
    {
        m_robotContainer.matchInit();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new MyAutoCommand();
         * break; case "Default Auto": default: autonomousCommand = new ExampleCommand();
         * break; }
         */

        //schedule the autonomous command (example)
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }

        m_robotContainer.autonomousInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.teleopInit();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.testInit();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {
        m_robotContainer.testPeriodic();

        
    }


}

