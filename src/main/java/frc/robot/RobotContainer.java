// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Konstants.ClimbConstants.kKrakenSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSuperSpeed;

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
import com.pathplanner.lib.path.PathPlannerPath;

import static frc.robot.Ports.DriverPorts.kDriver;
import static frc.robot.Ports.OperatorPorts.kOperator;

//import choreo.auto.AutoChooser;
//import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;
import frc.robot.bindings.ClimbBinder;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK25VisionBinder;
import frc.robot.bindings.AutoScoringBinder;
import frc.robot.bindings.RevBindings;
// import frc.robot.bindings.SK25ElevatorBinder;
//import frc.robot.utils.SK25AutoBuilder;
import frc.robot.bindings.SK25EndEffectorBinder;
import frc.robot.bindings.SK25LightsBinder;
// import frc.robot.bindings.SK25ScoringBinder;
import frc.robot.bindings.SKSwerveBinder;
import frc.robot.commands.AlignToReefTag.Target;
import frc.robot.commands.GoToSource.GoToBargeSourceFromN;
import frc.robot.commands.GoToSource.GoToBargeSourceFromNW;
import frc.robot.commands.GoToSource.GoToBargeSourceFromS;
import frc.robot.commands.GoToSource.GoToBargeSourceFromSW;
import frc.robot.commands.GoToSource.GoToProcessorSourceFromNE;
import frc.robot.commands.GoToSource.GoToProcessorSourceFromS;
import frc.robot.commands.GoToSource.GoToProcessorSourceFromSE;
import frc.robot.commands.EndEffectorButtonCommand;
import frc.robot.commands.commandGroups.AlignToReefComboAuton;
import frc.robot.commands.commandGroups.LineupCombo;
import frc.robot.commands.commandGroups.StationCombo;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
//import frc.robot.subsystems.Configs.CoralSubsystem;
import frc.robot.subsystems.SK25Climb;
import frc.robot.subsystems.SK25Elevator;
import frc.robot.subsystems.vision.SK25Vision;

import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SK25Lights;
import frc.robot.subsystems.SKSwerve;
import frc.robot.utils.SubsystemControls;
import frc.robot.utils.files.Elastic;
import frc.robot.utils.files.Elastic.Notification.NotificationLevel;
import frc.robot.utils.filters.FilteredJoystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends Robot{

    // private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // "MaxSpeed"


    // private void configurePhoenixTelemetry() {

    //     m_swerve.get().registerTelemetry(logger::telemeterize);
    // }

  // The robot's subsystems and commands are defined here...

  public Optional<SKSwerve> m_swerveContainer = Optional.empty();
  public Optional<SK25Vision> m_visionContainer = Optional.empty();
  public Optional<SK25Elevator> m_elevatorContainer = Optional.empty();
  public Optional<CoralSubsystem> m_coralContainer = Optional.empty();
  public Optional<SK25Lights> m_lightsContainer = Optional.empty();
  public Optional<SK25Climb> m_climbContainer = Optional.empty();
  public Optional<SK25EndEffector> m_endEffectorContainer = Optional.empty();

  public static SK25Vision m_vision;
  public static SKSwerve m_swerve;
  public static SK25Elevator m_elevator;
  public static CoralSubsystem m_coral;
  public static SK25Lights m_lights;
  public static SK25Climb m_climb;
  public static SK25EndEffector m_endEffector;

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
                m_lightsContainer = Optional.of(new SK25Lights());
                m_lights = m_lightsContainer.get();
            }
            if(subsystems.isElevatorPresent())
            {
                m_elevatorContainer = Optional.of(new SK25Elevator());
                m_elevator = m_elevatorContainer.get();
            }
            if(subsystems.isSwervePresent()) {
                m_swerveContainer = Optional.of(Konstants.TunerConstants.createDrivetrain());
                m_swerve = m_swerveContainer.get(); // Returns new SKSwerve
            }
            if(subsystems.isVisionPresent() && subsystems.isSwervePresent()) {
                m_visionContainer = Optional.of(new SK25Vision(m_swerveContainer));
                m_vision = m_visionContainer.get();
            }
            if(subsystems.isEndEffectorPresent())
            {
                m_endEffectorContainer = Optional.of(new SK25EndEffector());
                m_endEffector = m_endEffectorContainer.get();
            }
            if(subsystems.isClimbPresent()) {
                m_climbContainer = Optional.of(new SK25Climb());
                m_climb = m_climbContainer.get();
            }
            if(subsystems.isCoralSubsystemPresent()) {
                m_coralContainer = Optional.of(new CoralSubsystem());
                m_coral = m_coralContainer.get();
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
        buttonBinders.add(new SKSwerveBinder(m_swerveContainer, m_elevatorContainer));
        // buttonBinders.add(new SK25ElevatorBinder(m_elevator));
        buttonBinders.add(new SK25LightsBinder(m_lightsContainer));
        buttonBinders.add(new RevBindings(m_coralContainer));

        // Adding all the binding classes to the list
        buttonBinders.add(new ClimbBinder(m_climbContainer));
        buttonBinders.add(new SK25EndEffectorBinder(m_endEffectorContainer, m_coralContainer));
        // buttonBinders.add(new SK25ScoringBinder(m_endEffector, m_elevator));
        buttonBinders.add(new SK25VisionBinder(m_visionContainer, m_swerveContainer));
        buttonBinders.add(new AutoScoringBinder(m_visionContainer, m_swerveContainer, m_coralContainer, m_endEffectorContainer));

        // Traversing through all the binding classes to actually bind the buttons
        for (CommandBinder subsystemGroup : buttonBinders)
        {
            subsystemGroup.bindButtons();
        }

    }


    public void configurePathPlannerCommands()
    {
        if (m_swerveContainer.isPresent())
        {
            if(m_visionContainer.isPresent()) {
                NamedCommands.registerCommand("AlignToLeftReefCommandAuto", new AlignToReefComboAuton(
                                Target.LEFT, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("AlignToRightReefCommandAuto", new AlignToReefComboAuton(
                                Target.RIGHT, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("AlignToCenterReefCommandAuto", new AlignToReefComboAuton(
                                Target.CENTER, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("AlignToLeftReefCommandAuton", new AlignToReefComboAuton(
                                Target.LEFT, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("AlignToRightReefCommandAuton", new AlignToReefComboAuton(
                                Target.RIGHT, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("AlignToCenterReefCommandAuton", new AlignToReefComboAuton(
                                Target.CENTER, 
                                SK25Vision.DriveToPose.getConfig(), 
                                SK25Vision.RotateToPose.getConfig(), 
                                m_vision, 
                                m_swerve));
                NamedCommands.registerCommand("GoToBargeSourceFromSW", new GoToBargeSourceFromSW());
                NamedCommands.registerCommand("GoToBargeSourceFromN", new GoToBargeSourceFromN());
                NamedCommands.registerCommand("GoToBargeSourceFromNW", new GoToBargeSourceFromNW());
                NamedCommands.registerCommand("GoToBargeSourceFromS", new GoToBargeSourceFromS());

                NamedCommands.registerCommand("GoToProcessorSourceFromNE", new GoToProcessorSourceFromNE());
                NamedCommands.registerCommand("GoToProcessorSourceFromSE", new GoToProcessorSourceFromSE());
                NamedCommands.registerCommand("GoToProcessorSourceFromS", new GoToProcessorSourceFromS());

            }
            if (m_endEffectorContainer.isPresent())
            {
                SK25EndEffector effector = m_endEffectorContainer.get();


                //Roller Commands

                //NamedCommands.registerCommand("IntakeAutoCommand", new IntakeAutoCommand(EndEffectorPosition.kIntakePositionAngle, effector));
                NamedCommands.registerCommand("IntakeAutoCommand", effector.runRollerCommand(kRollerSpeed));
                NamedCommands.registerCommand("IntakeAlgaeAutoCommand", effector.runRollerCommand(kRollerSuperSpeed));
                NamedCommands.registerCommand("ExtakeAutoCommand", effector.runRollerCommand(-kRollerSpeed));
                NamedCommands.registerCommand("L4ExtakeAutoCommand", Commands.parallel(
                    new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
                    effector.runRollerCommand(-kRollerSpeed)));

                if(m_coralContainer.isPresent())
                {
                    CoralSubsystem elevator = m_coralContainer.get();

                    

                    //||||||||||  DONT CHANGE UNTIL OTHER COMMANDS WORK!  |||||||||||\\

                    NamedCommands.registerCommand("ElevatorTroughPositionCommand",
                        Commands.parallel(
                            elevator.setSetpointCommand(Setpoint.kLevel1),
                            new EndEffectorButtonCommand(EndEffectorPosition.kIntakePositionAngle, effector),
                            Commands.sequence(Commands.waitSeconds(1), effector.runRollerCommand(-0.3))   //correct extake directoin
                        )
                    );

                    //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\



                    //Score Commands

                    NamedCommands.registerCommand("TroughScoreCombo", new LineupCombo(
                        Setpoint.kLevel1,
                        elevator,
                        EndEffectorPosition.kTroughPositionAngle,
                        effector));

                    NamedCommands.registerCommand("L2ScoreCombo", new LineupCombo(
                        Setpoint.kLevel2,
                        elevator,
                        EndEffectorPosition.kLowPositionAngle,
                        effector));

                    NamedCommands.registerCommand("L3ScoreCombo", new LineupCombo(
                        Setpoint.kLevel3,
                        elevator,
                        EndEffectorPosition.kMiddleAngle,
                        effector));

                    NamedCommands.registerCommand("L4ScoreCombo", new LineupCombo(
                        Setpoint.kLevel4,
                        elevator,
                        EndEffectorPosition.kTopPositionAngle,
                        effector));

                    NamedCommands.registerCommand("NetScoreCombo", new LineupCombo(
                        Setpoint.kNet,
                        elevator,
                        EndEffectorPosition.kNetAngle,
                        effector));


                    //Pickup Commands

                    NamedCommands.registerCommand("StationPickupCombo", new LineupCombo(
                        Setpoint.kIntake,
                        elevator,
                        EndEffectorPosition.kIntake,
                        effector));

                    NamedCommands.registerCommand("LowAlgaePickupCombo", new LineupCombo(
                        Setpoint.kLowAlgae,
                        elevator,
                        EndEffectorPosition.kLowAlgae,
                        effector));

                    NamedCommands.registerCommand("HighAlgaePickupCombo", new LineupCombo(
                        Setpoint.kHighAlgae,
                        elevator,
                        EndEffectorPosition.kHighAlgae,
                        effector));

                    //Station Commands

                    NamedCommands.registerCommand("StationWaitCombo", new StationCombo(
                        Setpoint.kIntake,
                        elevator,
                        EndEffectorPosition.kIntake,
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
        return Commands.sequence(Commands.waitSeconds(0.01), autoCommandSelector.getSelected());
    }

    


    public void testPeriodic()
    {
        if(m_lightsContainer.isPresent())
        {
            m_lightsContainer.get().testPeriodic();
        }
        if(m_elevatorContainer.isPresent())
        {
            m_elevatorContainer.get().testPeriodic();
        }
        if(m_endEffectorContainer.isPresent())
        {
            m_endEffectorContainer.get().testPeriodic();
        }
        // if(m_coral.isPresent())
        // {
        //     m_coral.get().testPeriodic();
        // }
    }
    public void testInit(){
        if(m_lightsContainer.isPresent())
        {
            m_lightsContainer.get().testInit();
        }
        if(m_elevatorContainer.isPresent())
        {
            m_elevatorContainer.get().testInit();
        }
        if(m_endEffectorContainer.isPresent())
        {
            m_endEffectorContainer.get().testInit();
            m_endEffectorContainer.get().resetEncoder();
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

    @Override
    public void teleopPeriodic()
    {
    }


    public void teleopInit()
    {

    }

    public void autonomousInit()
    {
        if(m_endEffectorContainer.isPresent())
        {
            m_endEffectorContainer.get().leave();
        }

    } 
}
