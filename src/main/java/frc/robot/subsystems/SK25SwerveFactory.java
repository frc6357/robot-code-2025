package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SK25SwerveFactory {
    
    SK25SwerveModule frontLeftModule;
    SK25SwerveModule frontRightModule;
    SK25SwerveModule backLeftModule;
    SK25SwerveModule backRightModule;
    private SK25SwerveModule[] modules;
    Pigeon2 bird;

    Pose2d startingPose;
    SwerveDriveKinematics m_kinematics;
    SwerveModulePosition[] m_Positions;
    SwerveDriveOdometry m_odometry;

    public Rotation2d currentAngle;

    public SK25SwerveFactory(SK25SwerveModule frontLeftModule, SK25SwerveModule frontRightModule, SK25SwerveModule backLeftModule, SK25SwerveModule backRightModule, Pigeon2 bird)
    {
        //make new swerve module objects 
        this.frontLeftModule = frontLeftModule;
        this.frontRightModule = frontRightModule;
        this.backLeftModule = backLeftModule;
        this.backRightModule = backRightModule;
        this.modules = new SK25SwerveModule[]{
            this.frontLeftModule,
            this.frontRightModule,
            this.backLeftModule,
            this.backRightModule
        };
        this.bird = bird;
        //gets the current angle of the robot
        currentAngle = bird.getRotation2d();
        //reset gyro
        bird.reset();
    
        //position of the robot when code is deployed. starts at 0, 0.
        startingPose = new Pose2d();
    

        //kinematics object is used to convert chassisspeeds to swervedrive states
        m_kinematics = new SwerveDriveKinematics(
            frontLeftModule.moduleTranslation, 
            frontRightModule.moduleTranslation, 
            backLeftModule.moduleTranslation, 
            backRightModule.moduleTranslation);

        //creates new swerve modulve positions which deifne the distance traveled and rotation of each module
        m_Positions = new SwerveModulePosition[4];
        updateModulePositions();

        //creates a new swerve dirve odometry which uses the parameters to estimate the robots position on the feild (becomes less acurate over time)
        m_odometry = new SwerveDriveOdometry(m_kinematics, currentAngle, m_Positions, startingPose);
    }

    private void updateModulePositions() {
        m_Positions[0] = frontLeftModule.getLatestSwerveModulePosition();
        m_Positions[1] = frontRightModule.getLatestSwerveModulePosition();
        m_Positions[2] = backLeftModule.getLatestSwerveModulePosition();
        m_Positions[3] = backRightModule.getLatestSwerveModulePosition();
    }

    //The primary method to control the swerve drive via configuring all of its modules. takes polar coordinates of velocity (speed and direction)
    public void doSwerve(
        double desiredOmegaRadiansPerSecond,
        Translation2d desiredVelocityMetersPerSecond
    )
    {

        //chassis speeds defines the velocity and direction of the robot.
        //feildrelative constructor makes the chassis speed object feild relative by useing velocity in x (m/s), y (m/s), omega (radians), 
        //and robot angle (zero when facing directly away from your alliance station wall). Is a velocity vector.
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                desiredVelocityMetersPerSecond.getX(),
                desiredVelocityMetersPerSecond.getY(),
                desiredOmegaRadiansPerSecond
            ),
            currentAngle
        );

        //swerve module states
        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
        driveAllOpenLoop(desiredModuleStates);
        return;
        //position of the robot
        //Pose2d currentRobotPose = new Pose2d(robotTranslation, currentAngle);

        //transform objects turns pose2d objects into the new position of the robot as a pose2d. Transform is the transformation matrix from the old position.
        // Transform2d frontLeftTransformation = frontLeftModule.getTransformation(startingPose, getUpdatedOdometryPose());
        // Transform2d frontRightTransformation = frontRightModule.getTransformation(startingPose, getUpdatedOdometryPose());
        // Transform2d backLeftTransformation = backLeftModule.getTransformation(startingPose, getUpdatedOdometryPose());
        // Transform2d backRightTransformation = backRightModule.getTransformation(startingPose, getUpdatedOdometryPose());

        //make a swerve module state which takes the speed of the module and its rotation
        // SwerveModuleState frontLeftTargetState = new SwerveModuleState(translation, frontLeftModule.getModuleRotation());  //TODO: translation should (MABYE) be converted to the meters per second values
        // SwerveModuleState frontRightTargetState = new SwerveModuleState(translation, frontRightModule.getModuleRotation());
        // SwerveModuleState backLeftTargetState = new SwerveModuleState(translation, backLeftModule.getModuleRotation());
        // SwerveModuleState backRightTargetState = new SwerveModuleState(translation, backRightModule.getModuleRotation());
        // //array of module states for iteration
        // SwerveModuleState[] targetModuleStates = {frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState};

        // //set the target position of the swerve modules
        // frontLeftModule.setTargetState(desiredAngle, translation);
        // frontRightModule.setTargetState(desiredAngle, translation);
        // backLeftModule.setTargetState(desiredAngle, translation);
        // backRightModule.setTargetState(desiredAngle, translation);
    }

    //drive all modules
    private void driveAllOpenLoop(SwerveModuleState[] states) {
        for (int i = 0; i < 4; ++i) {
            SK25SwerveModule module = this.modules[i];
            SwerveModuleState desiredState = states[i];
            module.driveOpenLoop(desiredState);
        }
    }

    //updates the odometry
    public void updateOdometry() 
    {
        currentAngle = bird.getRotation2d();
        updateModulePositions();

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
