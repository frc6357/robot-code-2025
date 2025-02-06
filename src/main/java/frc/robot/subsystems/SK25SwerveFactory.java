package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK25SwerveFactory extends SubsystemBase{
    
    SK25SwerveModule frontLeftModule;
    SK25SwerveModule frontRightModule;
    SK25SwerveModule backLeftModule;
    SK25SwerveModule backRightModule;

    public Rotation2d currentAngle;

    public SK25SwerveFactory(SK25SwerveModule frontLeftModule, SK25SwerveModule frontRightModule, SK25SwerveModule backLeftModule, SK25SwerveModule backRightModule, Pigeon2 bird)
    {
        //make new swerve module objects 
        this.frontLeftModule = frontLeftModule;
        this.frontLeftModule = frontRightModule;
        this.frontLeftModule = backLeftModule;
        this.frontLeftModule = backRightModule;
        //gets the current angle of the robot
        currentAngle = bird.getRotation2d();
        //reset gyro
        bird.reset();
    }

    
    //position of the robot when code is deployed. starts at 0, 0.
    Pose2d startingPose = new Pose2d();
    

    //kinematics object is used to convert chassisspeeds to swervedrive states
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        frontLeftModule.moduleTranslation, 
        frontRightModule.moduleTranslation, 
        backLeftModule.moduleTranslation, 
        backRightModule.moduleTranslation);

    //creates new swerve modulve positions which deifne the distance traveled and rotation of each module
    SwerveModulePosition[] m_Positions = {
        frontLeftModule.getSwerveModulePosition(), 
        frontRightModule.getSwerveModulePosition(), 
        backLeftModule.getSwerveModulePosition(), 
        backRightModule.getSwerveModulePosition()};

    //creates a new swerve dirve odometry which uses the parameters to estimate the robots position on the feild (becomes less acurate over time)
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, currentAngle, m_Positions, startingPose);


    //The primary method to control the swerve drive via configuring all of its modules.
    public void doSwerve(double theta, double translation, double rotation, double vX, double vY)
    {
        //chassis speeds defines the velocity and direction of the robot.
        //feildrelative constructor makes the chassis speed object feild relative by useing velocity in x (m/s), y (m/s), omega (radians), 
        //and robot angle (zero when facing directly away from your alliance station wall). Is a velocity vector.
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, theta, currentAngle);

        //translation2d object of the robot
        //Translation2d robotTranslation = new Translation2d(translation, theta);

        //swerve module states
        SwerveModuleState[] currentModuleStates = m_kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveModuleState fLState = currentModuleStates[0];
        SwerveModuleState fRState = currentModuleStates[1]; 
        SwerveModuleState bLState = currentModuleStates[2]; 
        SwerveModuleState bRState = currentModuleStates[3]; 

        //position of the robot
        //Pose2d currentRobotPose = new Pose2d(robotTranslation, currentAngle);

        //transform objects turns pose2d objects into the new position of the robot as a pose2d. Transform is the transformation matrix from the old position.
        Transform2d frontLeftTransformation = frontLeftModule.getTransformation(startingPose, getUpdatedOdometryPose());
        Transform2d frontRightTransformation = frontRightModule.getTransformation(startingPose, getUpdatedOdometryPose());
        Transform2d backLeftTransformation = backLeftModule.getTransformation(startingPose, getUpdatedOdometryPose());
        Transform2d backRightTransformation = backRightModule.getTransformation(startingPose, getUpdatedOdometryPose());

        //make a swerve module state which takes the speed of the module and its rotation
        SwerveModuleState frontLeftTargetState = new SwerveModuleState(translation, frontLeftModule.getModuleRotation());  //TODO: translation should (MABYE) be converted to the meters per second values
        SwerveModuleState frontRightTargetState = new SwerveModuleState(translation, frontRightModule.getModuleRotation());
        SwerveModuleState backLeftTargetState = new SwerveModuleState(translation, backLeftModule.getModuleRotation());
        SwerveModuleState backRightTargetState = new SwerveModuleState(translation, backRightModule.getModuleRotation());
        //array of module states for iteration
        SwerveModuleState[] targetModuleStates = {frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState};
       
        //decrease the error in turning by preventing misdirection or oversteer/understeer
        frontLeftModule.decreaseError(fLState, currentAngle);
        frontRightModule.decreaseError(fRState, currentAngle);
        backLeftModule.decreaseError(bLState, currentAngle);
        backRightModule.decreaseError(bRState, currentAngle);

        //set the target position of the swerve modules
        frontLeftModule.setTargetState(Radians.of(theta), translation);
        frontRightModule.setTargetState(Radians.of(theta), translation);
        backLeftModule.setTargetState(Radians.of(theta), translation);
        backRightModule.setTargetState(Radians.of(theta), translation);
    }

    //updates the odometry
    public void updateOdometry() 
    {
        m_odometry.update(currentAngle, m_Positions);
    }

    //updates the odometry and returns the updated pose
    private Pose2d getUpdatedOdometryPose()
    {
        return m_odometry.update(currentAngle, m_Positions);
    }

    /**
     * Gets the current position of the robot from the odometry update.
     * @return The new robot position.
     */
    public Pose2d getRobotPose()
    {
        return getUpdatedOdometryPose();
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
        updateOdometry();
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
        updateOdometry();
    }
}
