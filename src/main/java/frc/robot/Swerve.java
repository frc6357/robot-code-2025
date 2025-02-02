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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.utils.SK25AutoBuilder;

public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements NTSendable, Subsystem {
    private SwerveConfig config;
    //private Notifier simNotifier = null;
    private double lastSimTime;
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
        configurePathPlanner();

        rotationController = new RotationController(config);

        // TODO: Review data communication methods (for apps like Shuffleboard, Elastic, etc.)
        SendableRegistry.add(this, "Swerve Drive");
        SmartDashboard.putData(this);
        Robot.add(this)
    }
    
}
