package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kBackLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kBackRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderOffsetDouble;
import static frc.robot.Konstants.SwerveConstants.kFrontRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kPigeonID;

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

    public Pigeon2 m_gyro;
    Rotation2d currentAngle;

    public SK25SwerveFactory()
    {
        //make new swerve module objects 
        frontLeftModule = new SK25SwerveModule(kFrontLeftDriveMotorID, kFrontLeftTurnMotorID, kFrontLeftEncoderID, kFrontLeftEncoderOffsetDouble);
        frontRightModule = new SK25SwerveModule(kFrontRightDriveMotorID, kFrontRightTurnMotorID, kFrontRightEncoderID, kFrontRightEncoderOffsetDouble);
        backLeftModule = new SK25SwerveModule(kBackLeftDriveMotorID, kBackLeftTurnMotorID, kBackLeftEncoderID, kBackLeftEncoderOffsetDouble);
        backRightModule = new SK25SwerveModule(kBackRightDriveMotorID, kBackRightTurnMotorID, kBackRightEncoderID, kBackRightEncoderOffsetDouble);
        //make a new Pigeon object which gives access to gyro positions
        m_gyro = new Pigeon2(kPigeonID);
        //gets the current angle of the robot
        currentAngle = m_gyro.getRotation2d();
        //reset gyro
        m_gyro.reset();
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

    //update the position using the current angle from the pigeon and the current positions of the swerve modules
    Pose2d updatedRobotPose = m_odometry.update(currentAngle, m_Positions);


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
        SwerveModuleState[] m_states = m_kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveModuleState fLState = m_states[0];
        SwerveModuleState fRState = m_states[1]; 
        SwerveModuleState bLState = m_states[2]; 
        SwerveModuleState bRState = m_states[3]; 

        //position of the robot
        //Pose2d currentRobotPose = new Pose2d(robotTranslation, currentAngle);

        //transform objects turns pose2d objects into the new position of the robot as a pose2d. Transform is the transformation matrix from the old position.
        Transform2d frontLeftTransformation = frontLeftModule.getTransformation(startingPose, updatedRobotPose);
        Transform2d frontRightTransformation = frontRightModule.getTransformation(startingPose, updatedRobotPose);
        Transform2d backLeftTransformation = backLeftModule.getTransformation(startingPose, updatedRobotPose);
        Transform2d backRightTransformation = backRightModule.getTransformation(startingPose, updatedRobotPose);

        //make a swerve module state which takes the speed of the module and its rotation
        SwerveModuleState frontLeftTargetState = new SwerveModuleState(translation, frontLeftModule.getModuleRotation());  //TODO: translation should (MABYE) be converted to the meters per second values
        SwerveModuleState frontRightTargetState = new SwerveModuleState(translation, frontRightModule.getModuleRotation());
        SwerveModuleState backLeftTargetState = new SwerveModuleState(translation, backLeftModule.getModuleRotation());
        SwerveModuleState backRightTargetState = new SwerveModuleState(translation, backRightModule.getModuleRotation());
        //array of module states for iteration
        SwerveModuleState[] moduleStates = {frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState};
       
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

    /**
     * Gets the current position of the robot from the odometry update.
     * @return The new robot position.
     */
    public Pose2d getRobotPose()
    {
        return updatedRobotPose;
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
}
