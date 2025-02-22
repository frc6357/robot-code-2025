// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.bindings.CommandBinder;
import frc.robot.bindings.SK25LightsBinder;
import frc.robot.bindings.SK25SwerveBinder;
//import frc.robot.bindings.TempSwerveBinder;
//import frc.robot.bindings.TempSwerveBinder;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SK25Lights;
import frc.robot.subsystems.SK25SwerveFactory;
//import frc.robot.subsystems.TempSwerve;
//import frc.robot.subsystems.TempSwerve;
//import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
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




    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private void configurePhoenixBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }



  // The robot's subsystems and commands are defined here...
  private Optional<SK25Lights> m_lights = Optional.empty();
  private Optional<SK25SwerveFactory> m_swerve = Optional.empty();

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

    //configures phoenix swerve bindings
    //configurePhoenixBindings();
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
                m_lights = Optional.of(new SK25Lights());
            }
            if(subsystems.isSwervePresent())
            {
                m_swerve = Optional.of(new SK25SwerveFactory());
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
        // if(m_swerve.isPresent())
        // {
        //    // NamedCommands.registerCommand("DefaultSwerveCommand", getAutonomousCommand());

        //     //Register commands for use in auto
        //     //NamedCommands.registerCommand("StartLauncherCommand", new LaunchCommandAuto(kLauncherLeftSpeed, kLauncherRightSpeed, launcher));
            
        // }

        // if(m_swerve.isPresent()){
            
        //     // Configures the autonomous paths and smartdashboard chooser
            
        //     //SK25AutoBuilder.setAutoNames(autoList);
        //     //autoCommandSelector = SK25AutoBuilder.buildAutoChooser("P4_Taxi");
        //     //SmartDashboard.putData("Auto Chooser", autoCommandSelector);
        // }
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
