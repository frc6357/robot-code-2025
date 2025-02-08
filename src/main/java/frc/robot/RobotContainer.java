// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK25ElevatorBinder;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.PracticeSwerve;
import frc.robot.utils.SK25AutoBuilder;
import frc.robot.utils.SubsystemControls;
import frc.robot.utils.filters.FilteredJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Optional<ExampleSubsystem> mySubsystem = Optional.empty();
  private Optional<PracticeSwerve> m_PracticeSwerve = Optional.empty();
  private Optional<SK25Elevator>    elevatorSubsystem    = Optional.empty();

  // The list containing all the command binding classes
  private List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();

  // The class used to create all PathPlanner Autos
  // private SK23AutoGenerator autoGenerator;
  // An option box on shuffleboard to choose the auto path
  SendableChooser<Command> autoCommandSelector = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlanner();

    // Configure the trigger bindings
    configureButtonBindings();
  }

  /**
     * Will create all the optional subsystems using the json file in the deploy directory
     */
    private void configureSubsystems()
    {
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();

        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Konstants.SUBSYSTEMFILE));
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);

            // Instantiating subsystems if they are present
            // This is decided by looking at Subsystems.json
            if(subsystems.isExamplePresent())
            {
                mySubsystem = Optional.of(new ExampleSubsystem());
            }
            if(subsystems.isElevatorPresent())
            {
                elevatorSubsystem = Optional.of(new SK25Elevator());
            }
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }
    }

  /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link FilteredJoystick}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {

        // Adding all the binding classes to the list
        buttonBinders.add(new SK25ElevatorBinder(elevatorSubsystem));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }

    private void configurePathPlanner()
    {
        if(m_PracticeSwerve.isPresent())
        {
                ExampleSubsystem subsystem = mySubsystem.get();
                
                NamedCommands.registerCommand("ExampleCommand", new ExampleCommand(subsystem));


            //Register commands for use in auto
            //NamedCommands.registerCommand("StartLauncherCommand", new LaunchCommandAuto(kLauncherLeftSpeed, kLauncherRightSpeed, launcher));
            
        }

        if(m_PracticeSwerve.isPresent()){
            
            // Configures the autonomous paths and smartdashboard chooser
            
            //SK25AutoBuilder.setAutoNames(autoList);
            autoCommandSelector = SK25AutoBuilder.buildAutoChooser("P4_Taxi");
            //SmartDashboard.putData("Auto Chooser", autoCommandSelector);
        }
    }

  /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoCommandSelector.getSelected();
    }

    public void testPeriodic(){
        if(mySubsystem.isPresent())
        {
            mySubsystem.get().testPeriodic();
        }
        if(elevatorSubsystem.isPresent())
        {
            elevatorSubsystem.get().testPeriodic();
        }
    }
    public void testInit(){
        if(mySubsystem.isPresent())
        {
            mySubsystem.get().testInit();
        }
        if(elevatorSubsystem.isPresent())
        {
            elevatorSubsystem.get().testInit();
        }
    }

    public void matchInit()
    {
        if (elevatorSubsystem.isPresent())
        {
            //SK25Elevator elevator = elevatorSubsystem.get();
            //TODO Add this back :)
            //elevator.setRightTargetHeight(0.0);
            //elevator.setLeftTargetHeight(0.0);
        }
    }

    public void teleopInit()
    {
       
    }
    public void autonomousInit()
    {
     
    }
}
