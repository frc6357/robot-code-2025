package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveInverted;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveInverted;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kChassisLength;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveInverted;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveInverted;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kJoystickDeadband;
import static frc.robot.Konstants.SwerveConstants.kMaxAngularRate;
import static frc.robot.Konstants.SwerveConstants.kSpeedAt12VoltsMeterPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.RotationController;
import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
import frc.robot.utils.Field;
import frc.robot.utils.Util;

public class TempSwerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem
{
    //stuff
    private SwerveConstantsConfigurator config = new SwerveConstantsConfigurator();

    //The phoenix swerveDriveState object contains information about the swerve modules, including wpilib classes
    public SwerveDriveState drivetrainState = this.getState();
    //module positions
    public SwerveModulePosition[] positions = drivetrainState.ModulePositions;
    //states
    public SwerveModuleState[] currentStates = drivetrainState.ModuleStates;
    public SwerveModuleState[] targetStates = drivetrainState.ModuleStates;
    //pose
    public Pose2d robotPose = drivetrainState.Pose;
    //chassisspeeds
    public ChassisSpeeds speeds = drivetrainState.Speeds;
    //raw heading
    public Rotation2d rawHeading = drivetrainState.RawHeading;

    public TempModule fL;
    public TempModule fR;
    public TempModule bL;
    public TempModule bR;

    public Pigeon2 bird;

    private final SwerveRequest.FieldCentric fieldCentricRequest;

    public RotationController controller;

    public SwerveDriveOdometry odometry;
    
 
    //constructor
    public TempSwerve(SwerveConstantsConfigurator config)
    {
        //TODO: new motors and encoder why???
        super(TalonFX::new, TalonFX::new, CANcoder::new, config.getDrivetrainConstants(), config.frontLeft, config.frontRight, config.backLeft, config.backRight);
        this.config = config;
      
    
        fL = new TempModule(config.frontLeft, kFrontLeftDriveInverted, kFrontLeftEncoderOffsetRadians, m_drivetrainId, 0);
        fR = new TempModule(config.frontRight, kFrontRightDriveInverted, kFrontRightEncoderOffsetRadians, m_drivetrainId, 1);
        bL = new TempModule(config.backLeft, kBackLeftDriveInverted, kBackLeftEncoderOffsetRadians, m_drivetrainId, 2);
        bR = new TempModule(config.backRight, kBackRightDriveInverted, kBackRightEncoderOffsetRadians, m_drivetrainId, 3);
        controller = new RotationController(config);
        bird = this.getPigeon2();

        odometry = new SwerveDriveOdometry(getDrivetrainKinematics(), getPigeonRotation(), positions, new Pose2d());     //TODO: alter initial pose mabye

        fieldCentricRequest = new SwerveRequest.FieldCentric()
                //.withDeadband(kSpeedAt12VoltsMeterPerSecond.in(MetersPerSecond) * kJoystickDeadband)    //TODO: use cubicDeadband filter instead
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                //.withRotationalDeadband(kMaxAngularRate * kJoystickDeadband)  
    }


    //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

    private void updateModulePositions()
    {
        SwerveModulePosition[] newPositions = getModulePositions();
        positions[0] = newPositions[0];
        positions[1] = newPositions[1];
        positions[2] = newPositions[2];
        positions[3] = newPositions[3];
    }


    //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
    public void normalDrive(double xSpeed, double ySpeed, double rotation)
    {
        this.setControl(fieldCentricRequest.withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotation));
        
        // fL.driveOpenLoop(fL.getTargetModuleState());
        // fR.driveOpenLoop(fR.getTargetModuleState());
        // bL.driveOpenLoop(bL.getTargetModuleState());
        // bR.driveOpenLoop(bR.getTargetModuleState());
    }
    

    //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
    
    double calculateRotationController(DoubleSupplier targetRadians) {
        return controller.calculate(targetRadians.getAsDouble(), getRotationRadians());
    }

    private Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        return keepPoseOnField(pose);
    }

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

    private double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    public void aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        //resetTurnController();
        normalDrive(
            velocityX.getAsDouble(),
            velocityY.getAsDouble(),
            //rotate to the target from the current rotation
            calculateRotationController(targetRadians));
    }

    //|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\

    //getters

    private ChassisSpeeds getCurrentRobotChassisSpeeds()
    {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private Rotation2d getPigeonRotation()
    {
        return bird.getRotation2d();
    }

    private Rotation2d getOdomHeading()
    {
        return getRobotPose().getRotation();
    }

    private SwerveDriveKinematics getDrivetrainKinematics()
    {
        return this.getKinematics();
    }

    private Translation2d[] getModuleLocation()
    {
        return this.getModuleLocations();
    }

  //odometry

    

    public void updateOdometry() 
    {
        updateModulePositions();

        odometry.update(getPigeonRotation(), positions);
    }

    //updates the odometry and returns the updated pose
    private Pose2d getUpdatedOdometryPose()
    {
        return odometry.update(getPigeonRotation(), positions);
    }

   
    /**
     * Gets the position of the swerve modules on the feild using the module rotation and distance driven by the drive motor.
     * @return The object containing the module position.
     */
    public SwerveModulePosition[] getModulePositions()
    {
        //new swerve module position with the current distance driven and current module rotation
        return drivetrainState.ModulePositions;
    }
    




    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
        //updateOdometry();
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
        //updateOdometry();
    }
}
