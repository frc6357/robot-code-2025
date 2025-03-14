// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import choreo.auto.AutoChooser;
//import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.bindings.ClimbBinder;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.RevBindings;
import frc.robot.bindings.SK25ElevatorBinder;
//import frc.robot.utils.SK25AutoBuilder;
import frc.robot.bindings.SK25EndEffectorBinder;
import frc.robot.bindings.SK25LightsBinder;
// import frc.robot.bindings.SK25ScoringBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.commandGroups.LineupCombo;
import frc.robot.commands.commandGroups.ScoreCombo;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
//import frc.robot.subsystems.Configs.CoralSubsystem;
import frc.robot.subsystems.SK25Climb;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SK25Lights;
import frc.robot.subsystems.SKSwerve;
import frc.robot.utils.SubsystemControls;
import frc.robot.utils.filters.FilteredJoystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // "MaxSpeed"


    // private void configurePhoenixTelemetry() {

    //     m_swerve.get().registerTelemetry(logger::telemeterize);
    // }

  // The robot's subsystems and commands are defined here...
  public Optional<SK25Elevator> m_elevator = Optional.empty();
  public Optional<CoralSubsystem> m_coral = Optional.empty();
  public Optional<SK25Lights> m_lights = Optional.empty();
  public Optional<SKSwerve> m_swerve = Optional.empty();
  // private Optional<ExampleSubsystem> mySubsystem = Optional.empty();
  public Optional <SK25Climb> m_Climb = Optional.empty();
  public Optional<SK25EndEffector> m_endEffector = Optional.empty();

  // The list containing all the command binding classes
  public List<CommandBinder> buttonBinders = new ArrayList<CommandBinder>();

  SendableChooser<Command> autoCommandSelector;



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
    //eats memory super fast, not good
    // configurePhoenixTelemetry();
  
    autoCommandSelector = AutoBuilder.buildAutoChooser("Taxi");
    //set delete old files = true in build.gradle to prevent sotrage of unused orphans
    SmartDashboard.putData("Select an Auto", autoCommandSelector);
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
            if(subsystems.isEndEffectorPresent())
            {
                m_endEffector = Optional.of(new SK25EndEffector());
            }
            if(subsystems.isClimbPresent()) {
                m_Climb = Optional.of(new SK25Climb());
            }
            if(subsystems.isCoralSubsystemPresent()) {
                m_coral = Optional.of(new CoralSubsystem());
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
        buttonBinders.add(new SKSwerveBinder(m_swerve, m_elevator));
        buttonBinders.add(new SK25ElevatorBinder(m_elevator));
        buttonBinders.add(new SK25LightsBinder(m_lights));
        buttonBinders.add(new RevBindings(m_coral));

        // Adding all the binding classes to the list
        buttonBinders.add(new ClimbBinder(m_Climb));
        buttonBinders.add(new SK25EndEffectorBinder(m_endEffector));
        // buttonBinders.add(new SK25ScoringBinder(m_endEffector, m_elevator));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }


    // public void configurePathPlannerCommands()
    // {
    //     if (m_swerve.isPresent())
    //     {
    //         if (m_coral.isPresent() && m_endEffector.isPresent())
    //         {
    //             CoralSubsystem coral = m_coral.get();
    //             SK25EndEffector effector = m_endEffector.get();



    //             //||||||||||  DONT CHANGE UNTIL AFTER BELTON  |||||||||||\\

    //             NamedCommands.registerCommand("ElevatorTroughPositionCommand",
    //                 Commands.parallel(
    //                     coral.setSetpointCommand(Setpoint.kLevel1),
    //                     new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                     Commands.sequence(Commands.waitSeconds(4), effector.runRollerCommand(-0.3))   //correct extake directoin
    //                 )
    //             );

    //             //||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

                

    //             NamedCommands.registerCommand(
    //                 "TroughScoreCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel1),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "Level2ScoreCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel2),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kLowPositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "Level3ScoreCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel3),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kMiddleAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "Level4ScoreCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel4),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kTopPositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "LowAlgaePickupCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel1),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "HighAlgaePickupCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel1),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "NetScoreCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel1),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );

    //             NamedCommands.registerCommand(
    //                 "StationPickupCombo",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kLevel1),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
    //                 Commands.sequence(Commands.waitSeconds(2), effector.runRollerCommand(-0.4))
    //                 )
    //             );
                
    //             NamedCommands.registerCommand(
    //                 "ElevatorZeroPositionCommand",
    //                 Commands.parallel(coral.setSetpointCommand(Setpoint.kZero),
    //                 new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector)
    //                 )
    //             );
            

    //             NamedCommands.registerCommand("IntakeAutoCommand", new InstantCommand(() -> effector.runRoller(kRollerSpeed)));
    //             NamedCommands.registerCommand("ExtakeAutoCommand", effector.runRollerCommand(-kRollerSpeed));
    //         }
    //     }
    // }

    public void configurePathPlannerCommands()
    {
        if (m_swerve.isPresent())
        {
            if (m_endEffector.isPresent())
            {
                SK25EndEffector effector = m_endEffector.get();


                //Roller Commands

                NamedCommands.registerCommand("IntakeAutoCommand", new InstantCommand(() -> effector.runRoller(kRollerSpeed)));
                NamedCommands.registerCommand("ExtakeAutoCommand", effector.runRollerCommand(-kRollerSpeed));    //TODO: change to auto speeds in konstants


                if(m_coral.isPresent())
                {
                    CoralSubsystem elevator = m_coral.get();

                    

                    //||||||||||  DONT CHANGE UNTIL OTHER COMMANDS WORK!  |||||||||||\\

                    NamedCommands.registerCommand("ElevatorTroughPositionCommand",
                        Commands.parallel(
                            elevator.setSetpointCommand(Setpoint.kLevel1),
                            new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
                            Commands.sequence(Commands.waitSeconds(4), effector.runRollerCommand(-0.3))   //correct extake directoin
                        )
                    );

                    //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\



                    //Score Commands

                    NamedCommands.registerCommand("TroughScoreCombo", new ScoreCombo(
                        Setpoint.kLevel1,
                        elevator,
                        EndEffectorPosition.kIntakePositionAngle,
                        effector));

                    NamedCommands.registerCommand("L2ScoreCombo", new ScoreCombo(
                        Setpoint.kLevel2,
                        elevator,
                        EndEffectorPosition.kLowPositionAngle,
                        effector));

                    NamedCommands.registerCommand("L3ScoreCombo", new ScoreCombo(
                        Setpoint.kLevel3,
                        elevator,
                        EndEffectorPosition.kMiddleAngle,
                        effector));

                    NamedCommands.registerCommand("L4ScoreCombo", new ScoreCombo(
                        Setpoint.kLevel4,
                        elevator,
                        EndEffectorPosition.kTopPositionAngle,
                        effector));

                    NamedCommands.registerCommand("NetScoreCombo", new ScoreCombo(
                        Setpoint.kNet,
                        elevator,
                        EndEffectorPosition.kNetAngle,
                        effector));


                    //Pickup Commands

                    NamedCommands.registerCommand("StationPickupCombo", new ScoreCombo(
                        Setpoint.kIntake,
                        elevator,
                        EndEffectorPosition.kIntake,
                        effector));

                    NamedCommands.registerCommand("LowAlgaePickupCombo", new ScoreCombo(
                        Setpoint.kLowAlgae,
                        elevator,
                        EndEffectorPosition.kLowAlgae,
                        effector));

                    NamedCommands.registerCommand("HighAlgaePickupCombo", new ScoreCombo(
                        Setpoint.kHighAlgae,
                        elevator,
                        EndEffectorPosition.kHighAlgae,
                        effector));


                    //Zero Position Command

                    NamedCommands.registerCommand("ZeroPositionCommand", new LineupCombo(
                        Setpoint.kZero,
                        elevator,
                        EndEffectorPosition.kZeroPositionAngle,
                        effector));
                }
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
        if(m_endEffector.isPresent())
        {
            m_endEffector.get().testPeriodic();
        }
        // if(m_coral.isPresent())
        // {
        //     m_coral.get().testPeriodic();
        // }
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
        if(m_endEffector.isPresent())
        {
            m_endEffector.get().testInit();
            m_endEffector.get().resetEncoder();
        }
    }

    public void matchInit()
    {
        /*
        if (m_elevator.isPresent())
        {
            //SK25Elevator elevator = elevatorSubsystem.get();
            //elevator.setRightTargetHeight(0.0);
            //elevator.setLeftTargetHeight(0.0);
        }
        if(m_endEffector.isPresent())
        {
            SK25lendEffector endeffector = m_endEffector.get();
        }
        */
    }

    public void teleopInit()
    {
        m_swerve.ifPresent((swerve) -> swerve.seedFieldCentric());
    }

    public void autonomousInit()
    {
        if(m_endEffector.isPresent()) {
            m_endEffector.get().leave();
        }

    } 
}
