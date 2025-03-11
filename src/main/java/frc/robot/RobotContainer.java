// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SKSwerveBinder;

import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25Lights;

import frc.robot.utils.SubsystemControls;
import frc.robot.utils.filters.FilteredJoystick;
//import static frc.robot.subsystems.TempSwerve.config;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // "MaxSpeed"


    private void configurePhoenixTelemetry() {

        m_swerveContainer.get().registerTelemetry(logger::telemeterize);
    }



  // The robot's subsystems and commands are defined here...

  private Optional<SK25Elevator> m_elevatorContainer = Optional.empty();
  private Optional<SK25Lights> m_lightsContainer = Optional.empty();
  private Optional<SKSwerve> m_swerveContainer = Optional.empty();
  private Optional<SK25Vision> m_visionContainer = Optional.empty();

  public static SK25Elevator m_elevator;
  public static SK25Lights m_lights;
  public static SK25Vision m_vision;
  public static SKSwerve m_swerve;


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

    // Configures swerve telemetry
    // configurePhoenixTelemetry();

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

            if(subsystems.isLightsPresent())
            {
                m_lightsContainer = Optional.of(new SK25Lights());
                m_lights = m_lightsContainer.get();
            }
            if(subsystems.isElevatorPresent()) {
                m_elevatorContainer = Optional.of(new SK25Elevator());
                m_elevator = m_elevatorContainer.get();
            }
            if(subsystems.isSwervePresent()) {
                m_swerveContainer = Optional.of(TunerConstants.createDrivetrain());
                m_swerve = m_swerveContainer.get(); // Returns new SKSwerve
            }
            if(subsystems.isVisionPresent() && subsystems.isSwervePresent()) {
                m_visionContainer = Optional.of(new SK25Vision(m_swerveContainer));
                m_vision = m_visionContainer.get();
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
        buttonBinders.add(new SKSwerveBinder(m_swerveContainer));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }


    private void configurePathPlanner()
    {
        /* 
        if(m_Swerve.isPresent())
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
        */
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
        // if(m_swerve.isPresent())
        // {
        //     m_swerve.get().testPeriodic();
        // }
        if(m_lightsContainer.isPresent())
        {
            m_lightsContainer.get().testPeriodic();
        }
        if(m_elevatorContainer.isPresent())
        {
            m_elevatorContainer.get().testPeriodic();
        }
    }
    public void testInit(){
        // if(m_swerve.isPresent())
        // {
        //     m_swerve.get().testInit();
        // }
        if(m_lightsContainer.isPresent())
        {
            m_lightsContainer.get().testInit();
        }
        if(m_elevatorContainer.isPresent())
        {
            m_elevatorContainer.get().testInit();
        }
    }

    public void matchInit()
    {
        //if (elevatorSubsystem.isPresent())
        //{
            //SK25Elevator elevator = elevatorSubsystem.get();
            //TODO Add this back :)
            //elevator.setRightTargetHeight(0.0);
            //elevator.setLeftTargetHeight(0.0);
        //}
    }

    public void teleopInit()
    {
        
    }
    public void autonomousInit()
    {
     
    } 
}
