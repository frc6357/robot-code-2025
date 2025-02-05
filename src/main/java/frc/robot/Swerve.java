package frc.robot;

// Packages used for the mechanisms/motors for the swerve drivetrain itself
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Packages used for Pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// Packages used for positioning, kinematics, odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// Packages used for logging, communications, overall functionality
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.utils.SK25AutoBuilder;
import static frc.robot.Konstants.AutoConstants.*;

public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements NTSendable, Subsystem {
    private SwerveConfig config;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private RotationController rotationController;

    private boolean hasAppliedDriverPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    //Creates publishers for logging
    StructArrayPublisher<SwerveModuleState> currentPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("CurrentSwerveStates", SwerveModuleState.struct).publish();

    StructArrayPublisher<SwerveModuleState> targetPublisher = 
    NetworkTableInstance.getDefault().
    getStructArrayTopic("TargetSwerveStates", SwerveModuleState.struct).publish();

    StructPublisher<Rotation2d> odomPublisher = NetworkTableInstance.getDefault().
    getStructTopic("Rotation", Rotation2d.struct).publish();

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing constants for 
     * the drivetrain and the module configurations.
     */
    public Swerve(SwerveConfig config) {
        // Creates a Swerve Drivetrain using Phoenix6's SwerveDrivetrain class, passing the
        // properties of the swerve drive itself from SwerveConfig into the constructor.
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        config.getDrivetrainConstants(),
        config.getModules());

        this.config = config;
        setupPathPlanner();

        rotationController = new RotationController(config);

        // TODO: Review data communication methods (for apps like Shuffleboard, Elastic, etc.)
        SendableRegistry.add(this, "Swerve Drive");
        SmartDashboard.putData(this);
        Robot.add(this)
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(config.getKSimLoopPeriod());
    }

    public void setupPathPlanner() {
        resetPose(
                new Pose2d(
                        Units.feetToMeters(27.0),
                        Units.feetToMeters(27.0 / 2.0),
                        config.getBlueAlliancePerspectiveRotation()));
        double driveBaseRadius = .5; //or something
        for(var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        RobotConfig robotConfig = null;
        try {
            RobotConfig.fromGUISettings();
        }
        catch (Exception e){
            e.printStackTrace();
        }
    
    SK25AutoBuilder.configure(
        () -> this.getState().Pose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, driveFeedForwards) -> 
                this.setControl(
                    AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        kAutoPathConfig,
        robotConfig,
        // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Reference to this subsystem to set requirements
        );
  }
    
}
