// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kLowPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kMidPosition;
import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.kTopPosition;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK25ElevatorBinder;
import frc.robot.bindings.SK25LightsBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.commands.ElevatorButtonCommand;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25Lights;
import frc.robot.subsystems.SKSwerve;
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

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // "MaxSpeed"


    private void configurePhoenixTelemetry() {

        m_swerve.get().registerTelemetry(logger::telemeterize);
    }

  // The robot's subsystems and commands are defined here...
  private Optional<SK25Elevator> m_elevator = Optional.empty();
  private Optional<SK25Lights> m_lights = Optional.empty();
  public static Optional<SKSwerve> m_swerve = Optional.empty();

  // The list containing all the command binding classes
  private List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();

  SendableChooser<Command> autoCommandSelector;// = new SendableChooser<Command>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {

    // Creates all subsystems that are on the robot
    configureSubsystems();

    // sets up autos needed for pathplanner
    configurePathPlannerCommands();

    // Configure the trigger bindings
    configureButtonBindings();

    // Configures swerve telemetry
    configurePhoenixTelemetry();


    // Build an auto chooser. This will use Commands.none() as the default option.
    autoCommandSelector = SK25AutoBuilder.buildAutoChooser("Taxi");

    // set deleteOldFiles = true in build.gradle, or deleted autos will load still
    SmartDashboard.putData("Select An Auto", autoCommandSelector);
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

            if(subsystems.isLightsPresent())
            {
                m_lights = Optional.of(new SK25Lights());
            }
            if(subsystems.isElevatorPresent())
            {
                m_elevator = Optional.of(new SK25Elevator());
            }
            if(subsystems.isSwervePresent()) {
                m_swerve = Optional.of(TunerConstants.createDrivetrain()); // Returns new SKSwerve
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
        buttonBinders.add(new SKSwerveBinder(m_swerve));
        buttonBinders.add(new SK25ElevatorBinder(m_elevator));
        buttonBinders.add(new SK25LightsBinder(m_lights));
        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }


    public void configurePathPlannerCommands()
    {
        if (m_swerve.isPresent())
        {
            if (m_elevator.isPresent())
            {
                SK25Elevator elevator = m_elevator.get();

                NamedCommands.registerCommand("ElevatorLowPositionCommand", new ElevatorButtonCommand(kLowPosition, elevator));
                NamedCommands.registerCommand("ElevatorMidPositionCommand", new ElevatorButtonCommand(kMidPosition, elevator));
                NamedCommands.registerCommand("ElevatorHighPositionCommand", new ElevatorButtonCommand(kTopPosition, elevator));
            }
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * <p>
     * This method loads the auto when it is called, however, it is recommended
     * to first load your paths/autos when code starts, then return the
     * pre-loaded auto/path.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        //return new PathPlannerAuto(autoCommandSelector.getSelected());
        return autoCommandSelector.getSelected();
    }


    public void testPeriodic()
    {
        if(m_lights.isPresent())
        {
            m_lights.get().testPeriodic();
        }
        if(m_elevator.isPresent())
        {
            m_elevator.get().testPeriodic();
        }
    }
    public void testInit(){
        if(m_lights.isPresent())
        {
            m_lights.get().testInit();
        }
        if(m_elevator.isPresent())
        {
            m_elevator.get().testInit();
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
