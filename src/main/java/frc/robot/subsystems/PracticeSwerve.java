package frc.robot.subsystems;

import static frc.robot.Konstants.PracticeSwerveConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Remove suppresswarnings annotations for when using in context
//Remove suppresswarnings annotations for when using in context 
//Remove suppresswarnings annotations for when using in context
//Remove suppresswarnings annotations for when using in context
//Remove suppresswarnings annotations for when using in context

//defines the SwerveModule class which provides methods to manipulate the drive and turn motors within a module via current states of modules
@SuppressWarnings("unused")
class SwerveModule
{

    //declares drive and turn motors
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    //declares current swerve module state which allows control of a module's wheel velocity (speed and direction)
    private SwerveModuleState currentState;
    //declares target swerve module state to replace current state
    private SwerveModuleState targetState;
    //declares PID controllers: objects which create PID loops to stop the swerve from bouncing over and under the translation and rotation targets using tunable constans P, I, and D
    PIDController drivePIDController;
    PIDController turnPIDController;


    // new turning simulator with a flywheel system, motor count (including type), and the standard deviations the Sim is away from target. These are as parameters in that order.
    public FlywheelSim turningSim = new FlywheelSim(
        //createFlywheelSystem creates the system to be simulated using parameters in thsi order: the motor count (including type), moment of inertia, and gearing
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.004, 150.0/7.0),
        DCMotor.getFalcon500(1),
         0.0);

    //constructor
    public SwerveModule(int driveMotorPort, int turnMotorPort)
    {
        //initializes drive and turn motors
        driveMotor = new TalonFX(driveMotorPort);
        turnMotor = new TalonFX(turnMotorPort);
        //intializes the Swerve Module State. The SwerveModuleState() consturctor assigns default values of zero to speed and direction in m/s and radians
        currentState = new SwerveModuleState();

        //initialize PIDControllers
        drivePIDController = new PIDController(kDriveP, kDriveI, kDriveD);
        turnPIDController = new PIDController(kTurnP, kTurnI, kTurnD);
    }

    //gets the current state of the swerve module
    public SwerveModuleState getState()
    {
        return currentState;
    }

    //sets the current state of the swerve module
    public void setTargetState(SwerveModuleState newState)
    {
        targetState = newState;
    }

    //runs every 20 miliseconds
    public void periodic()
    {
        //updates the steering simulater, 0.02 is 20 miliseconds
        turningSim.update(0.02);

        //get the new simulated angle change since last update in radians
        double simulatedAngleDifferenceRadians = turningSim.getAngularVelocityRadPerSec() * 0.02;

        //update current state with new speed and angle
        currentState = new SwerveModuleState(
            currentState.speedMetersPerSecond,
            Rotation2d.fromDegrees(currentState.angle.getDegrees() + Units.radiansToDegrees(simulatedAngleDifferenceRadians))
        );
    }
}

//defines the PracticeSwerve class which is used to drive the robot by giving instruction to each SwerveModule
public class PracticeSwerve extends SubsystemBase{

    //array of all 4 swerve modules to allow iteration with for() loops
    SwerveModule swerveModules [] = new SwerveModule[4];

    //create swerve modules
    SwerveModule frontRightModule = new SwerveModule(kFrontRightDriveMotorId, kFrontRightSteerMotorId);
    SwerveModule frontLeftModule = new SwerveModule(kFrontLeftDriveMotorId, kFrontLeftSteerMotorId);
    SwerveModule backRightModule = new SwerveModule(kBackRightDriveMotorId, kBackRightSteerMotorId);
    SwerveModule backLeftModule = new SwerveModule(kBackLeftDriveMotorId, kBackLeftSteerMotorId);

    //width and length of swerve drive chassis in inches to be used for rotational targets of swerve modules
    double chassisWidth = Units.inchesToMeters(kChassisWidth);
    double chassisLength = Units.inchesToMeters(kChassisLength);
    //creates Translation2d objects with X and Y coordinates of wheels relative to the exact center of the robot (equal to the first and second parameters respectivley)
    //X is positive toward the front, Y is positive toward the left, divided by 2 to define distance from center
    Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d frontRightLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d backLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d backRightLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);

    //defines a kinematics object which takes chassis speed objects to return SwereveModuleStates
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
        );


    //constructor
    public PracticeSwerve()
    {
    }

    //set the speed of the chassis
    public void setChassisSpeed(ChassisSpeeds targetChassisSpeed)
    {
        //get the target states of the wheels
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(targetChassisSpeed);

        //group modules into array so all states can be set (speed and direction)
        frontLeftModule.setTargetState(newStates[0]);
        frontRightModule.setTargetState(newStates[1]);
        backLeftModule.setTargetState(newStates[2]);
        backRightModule.setTargetState(newStates[3]);

    }
    //periodic method runs everything within it every 20 milisceonds
    @Override
    public void periodic()
    {
       
        //Logging data for elastic dashboard
        //Ordered: FL, FR, BL, BR
        @SuppressWarnings("unused")
        double ModuleStates[] = {
            frontLeftModule.getState().angle.getDegrees(),     //gets angle in degrees of FL module
            frontLeftModule.getState().speedMetersPerSecond,   //gets speed in m/s of FL module
            frontRightModule.getState().angle.getDegrees(),    //gets angle in degrees of FR module
            frontRightModule.getState().speedMetersPerSecond,  //gets speed in m/s of FR module
            backLeftModule.getState().angle.getDegrees(),      //gets angle in degrees of BL module
            backLeftModule.getState().speedMetersPerSecond,    //gets speed in m/s of BL module
            backRightModule.getState().angle.getDegrees(),     //gets angle in degrees of BR module
            backRightModule.getState().speedMetersPerSecond,   //gets speed in m/s of BR module
        };

        //put the ModuleStates array into elastic dashboard here
    }





    
}
