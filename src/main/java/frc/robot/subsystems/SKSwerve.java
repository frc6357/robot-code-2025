package frc.robot.subsystems;

import static frc.robot.Konstants.AutoConstants.pathConfig;
import static frc.robot.Konstants.SwerveConstants.kChassisLength;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

//import choreo.Choreo.TrajectoryLogger;
//import choreo.auto.AutoFactory;
//import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.Field;
import frc.lib.utils.Util;
import frc.robot.Robot;
import frc.robot.Konstants.DriveConstants;
import frc.robot.subsystems.drive.GeneratedConstants;
import frc.robot.subsystems.drive.GeneratedDrivetrain;
import frc.robot.subsystems.drive.GeneratedTelemetry;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SKSwerve extends SubsystemBase {
    private SwerveDriveState lastReadState;
    private final GeneratedDrivetrain drivetrain = GeneratedConstants.createDrivetrain();
    private SwerveDrivePoseEstimator poseEstimator;
    private final GeneratedTelemetry telemetry = new GeneratedTelemetry(DriveConstants.kMaxSpeed);

    private Field2d elasticField = new Field2d();

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish();


    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    

    public SKSwerve() {
        lastReadState = drivetrain.getState();
        
        setupPoseEstimator();
        configureAutoBuilder();

    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), drivetrain.getState().ModulePositions);

        outputTelemetry();
    }

    public void outputTelemetry() {
		posePublisher.set(getRobotPose());
		telemetry.telemeterize(lastReadState);
		SmartDashboard.putData("Drive", this);
		elasticField.setRobotPose(getRobotPose());
		SmartDashboard.putData("Elastic Field 2D", elasticField);
	}

    @Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
				"Pitch Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityYDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Pitch Degrees",
				() -> drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees),
				null);

		builder.addDoubleProperty(
				"Roll Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityXDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Roll Degrees",
				() -> drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees),
				null);

		addModuleToBuilder(builder, 0);
		addModuleToBuilder(builder, 1);
		addModuleToBuilder(builder, 2);
		addModuleToBuilder(builder, 3);
	}

	private void addModuleToBuilder(SendableBuilder builder, int module) {
		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Volts",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Volts",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);
	}


    public GeneratedDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private void setupPoseEstimator() {
        poseEstimator = 
            new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(), 
                new Rotation2d(drivetrain.getPigeon2().getYaw().getValue()), 
                drivetrain.getState().ModulePositions, 
                new Pose2d());
    }

    private void setupPoseEstimatorWithStdDevs(Matrix<N3, N1> odomStdDevs, Matrix<N3, N1> visionStdDevs) {
        poseEstimator = 
            new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(), 
                new Rotation2d(drivetrain.getPigeon2().getYaw().getValue()), 
                drivetrain.getState().ModulePositions, 
                new Pose2d(),
                odomStdDevs, 
                visionStdDevs);
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to
     *                                 trust global measurements from vision less.
     *                                 This matrix is in the form [x, y, theta]ᵀ,
     *                                 with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /* 
     *
     * Autonomous
     * 
     */

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getRobotPose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                pathConfig,
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void simulateCollision() {
        if(!Robot.isReal()) {
            resetPose(getRobotPose().plus(new Transform2d(-0.4, 0.5, getRobotRotation().plus(Rotation2d.fromDegrees(30)))));
        }
    }

    /**
    * Set chassis speeds of robot to drive it robot oreintedly.
    * @param chassisSpeeds Chassis Speeds to set.
    */
    public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards ff)
    {
        SwerveRequest chassisSpeed = new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds);
        drivetrain.setControl(chassisSpeed);
    }

    // public void robotRelativeDrive(ChassisSpeeds speeds)
    // {
    //     final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
    //     this.applyRequest(() ->
    //             robotCentricDrive.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
    //                 .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
    //                 .withRotationalRate(speeds.omegaRadiansPerSecond)); // Drive counterclockwise with negative X (left)
    // }

    /* 
     *
     * Odemetry Methods 
     * 
     */

    /** 
     * Keep the robot on the field using the field length from Util by checking if the position is off 
     * the field, then replacing it with the correct position
     * @return The new pose after limiting out of field possibilities.
     */
    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = kChassisLength / 2;
        double x = pose.getX();
        double y = pose.getY();

        double newX = Util.limit(x, halfRobot, Field.getFieldLength() - halfRobot);
        double newY = Util.limit(y, halfRobot, Field.getFieldWidth() - halfRobot);

        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }
        return pose;
    }

    /**
     * 
     * @return The estimation of the robot's pose
     */
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
       // return pose;
    }

    public SwerveModuleState[] getModuleStates() {
        return drivetrain.getState().ModuleStates;
    }

    public Rotation2d getRobotRotation() {
        return getRobotPose().getRotation();
    }

    public Rotation2d getGyroRotation() {
        return drivetrain.getPigeon2().getRotation2d();
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getKinematics().toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getVelocity(boolean fieldRelative) {
        if(fieldRelative) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getGyroRotation());
        }
        else {
            return getRobotRelativeSpeeds();
        }
    }

    public void resetOrientation() {
        boolean flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
          if (flip) {
            drivetrain.resetRotation(Rotation2d.k180deg);
          } else {
            drivetrain.resetRotation(Rotation2d.kZero);
          }
    }

    /** Resets odometry to the given pose.
     * @param pose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d pose)
    {
        resetPose(pose);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        poseEstimator.resetPose(pose);
    }

    // public ChassisSpeeds getRobotSpeeds()
    //     {
    //         SwerveDriveKinematics m_kinematics = this.getKinematics();
    //         SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = this.getModules();
    //         SwerveModuleState[] currentStates = new SwerveModuleState[4];
    //         for (int i = 0; i < 4; i++)
    //         {
    //             SwerveModuleState phoenixModuleState = modules[i].getCurrentState();
    //             currentStates[i] = phoenixModuleState;
    //         }
    //         return m_kinematics.toChassisSpeeds(currentStates);
    //     }
}
