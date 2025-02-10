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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK25LightsBinder;
import frc.robot.bindings.SK25SwerveBinder;
import frc.robot.subsystems.SK25Lights;
import frc.robot.subsystems.SK25Swerve;
import frc.robot.bindings.SK25DriveBinder;
import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
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
  private Optional<SK25Lights> m_lights = Optional.empty();
  private Optional<SK25Swerve> m_swerve = Optional.empty();

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
    //configurePathPlanner();

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
            if(subsystems.isSwervePresent())
            {
                SwerveConstantsConfigurator config = new SwerveConstantsConfigurator();
                //m_Swerve = Optional.of(new SK25Swerve(config));            //TODO: resolve collision in robot container with swerve constructor differences
            }
            if(subsystems.isLightsPresent())
            {
                m_lights = Optional.of(new SK25Lights());
            }
            if(subsystems.isSwervePresent())
            {
                m_swerve = Optional.of(new SK25Swerve());
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
        buttonBinders.add(new SK25LightsBinder(m_lights));
        buttonBinders.add(new SK25SwerveBinder(m_swerve));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }


    private void configurePathPlanner()
    {
        if(m_swerve.isPresent())
        {
           // NamedCommands.registerCommand("DefaultSwerveCommand", getAutonomousCommand());

            //Register commands for use in auto
            //NamedCommands.registerCommand("StartLauncherCommand", new LaunchCommandAuto(kLauncherLeftSpeed, kLauncherRightSpeed, launcher));
            
        }

        if(m_swerve.isPresent()){
            
            // Configures the autonomous paths and smartdashboard chooser
            
            //SK25AutoBuilder.setAutoNames(autoList);
            //autoCommandSelector = SK25AutoBuilder.buildAutoChooser("P4_Taxi");
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

    public void testPeriodic()
    {
        if(m_swerve.isPresent())
        {
            m_swerve.get().testPeriodic();
        }
        if(m_lights.isPresent())
        {
            m_lights.get().testPeriodic();
        }
    }
    public void testInit(){
        if(m_swerve.isPresent())
        {
            m_swerve.get().testInit();
        }
        if(m_lights.isPresent())
        {
            m_lights.get().testInit();
        }
    }

    public void matchInit()
    {
    
    }

    public void teleopInit()
    {
       
    }
    public void autonomousInit()
    {
     
    } 
}
