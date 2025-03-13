package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.utils.konstantLib.wrappers.SKTrigger.INPUT_TYPE.*;
import static frc.robot.Konstants.kCANivoreName;
import static frc.robot.Konstants.kDefualtRioBusName;
import static frc.robot.Konstants.ElevatorConstants.kRightElevatorMotorID;
import static frc.robot.Konstants.EndEffectorConstants.kEndEffectorArmMotorID;
import static frc.robot.Konstants.EndEffectorConstants.kEndEffectorLaserCanID;
import static frc.robot.Konstants.EndEffectorConstants.kEndEffectorRollerMotorID;
import static frc.robot.Konstants.LightConstants.kCandleID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kPigeonID;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.LEFT_Y;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.LEFT_X;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.RIGHT_X;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.RIGHT_Y;
import static frc.robot.utils.konstantLib.SKController.ControllerType.XBOX;

import frc.robot.utils.konstantLib.CANPort;
import frc.robot.utils.konstantLib.SKController;
import frc.robot.utils.konstantLib.filters.FilteredAxis;
import frc.robot.utils.konstantLib.wrappers.SKTrigger;



public class Ports
{
    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    public static class DriverPorts 
    {
        /** The Driver Controller object.
         * <p>
         * The SKController type allows swapping between controller types
         * or control schemes by simply switching the ControllerType enum, which lets SKController
         * handle button remapping.*/
        public static final SKController kDriver = new SKController(XBOX, 0);
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kTranslationXPort = kDriver.getRawSKContorllerAxis(LEFT_Y);
        public static final FilteredAxis kTranslationYPort = kDriver.getRawSKContorllerAxis(LEFT_X);
        public static final FilteredAxis kVelocityOmegaPort = kDriver.getRawSKContorllerAxis(RIGHT_X);
        // public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> kDriver.getUnderlyingHIDController().getRawAxis(kLeftY.value));
        // public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> kDriver.getUnderlyingHIDController().getRawAxis(kLeftX.value));
        // public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> kDriver.getUnderlyingHIDController().getRawAxis(kRightX.value));
        
        // Driver Function Button (Activates secondary control scheme when held)
        public static final SKTrigger kDriveFn = kDriver.mapLeftShoulderButton();

        // Switch modes
        public static final SKTrigger kRobotCentric = kDriver.mapSelect();
        public static final SKTrigger kSlowMode = kDriver.mapRightShoulderButton();

        // Reset gyro
        public static final SKTrigger kResetGyroPos = kDriver.mapRightJoystickPress();

        //Climb
        public static final SKTrigger climbRaiseButton = kDriver.mapUpDPad();
        public static final SKTrigger climbLowerButton = kDriver.mapDownDPad();
        public static final SKTrigger climbStopButton = kDriver.mapRightDPad();
        public static final SKTrigger climbSlowButton = kDriver.mapLeftDPad();


        // public static final GenericHID kDriver = new FilteredXboxController(0).getHID();
        
        // // Filtered axis (translation & rotation)
        // public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftY.value));
        // public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftX.value));
        // public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> kDriver.getRawAxis(kRightX.value)); 

        // public static final SKTrigger climbRaiseButton = new SKTrigger(kDriver, 0, POV);
        // public static final SKTrigger climbLowerButton = new SKTrigger(kDriver, 180, POV);
        // public static final SKTrigger climbStopButton = new SKTrigger(kDriver, 90, POV);
        // public static final SKTrigger climbSlowButton = new SKTrigger(kDriver, 270, POV);
        
        // // Driver Function Button (Activates secondary control scheme when held)
        // public static final SKTrigger kDriveFn = new SKTrigger(kDriver, kLeftBumper.value, BUTTON);

        // // Switch modes
        // public static final SKTrigger kRobotCentricMode = new SKTrigger(kDriver, 180, POV); // Function Controlscheme (NOTE: This button is meant to be impossible to accidentally press)
        // public static final SKTrigger kSlowMode = new SKTrigger(kDriver, kRightBumper.value, BUTTON); // Function Controlscheme

        

        // // Reset gyro
        // public static final SKTrigger kResetGyroPos = new SKTrigger(kDriver, kRightStick.value, BUTTON);
    }

    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    public static class OperatorPorts
    {
        /** The Operator Controller object.
        * <p>
        * The SKController type allows swapping between controller types
        * or control schemes by simply switching the ControllerType enum, which lets SKController
        * handle button remapping.*/
        public static final SKController kOperator = new SKController(XBOX, 1);
        
        //Lights
        public static final SKTrigger kPartyMode = kOperator.mapStart();
        public static final SKTrigger kLightsToTeal = kOperator.mapSelect();

        //Coral Positions
        public static final SKTrigger kStationPos = kOperator.mapLeftShoulderButton();
        public static final SKTrigger kTroughPos = kOperator.mapDownDPad();
        public static final SKTrigger kL2BranchPos = kOperator.mapLeftDPad();
        public static final SKTrigger kL3BranchPos = kOperator.mapRightDPad();
        public static final SKTrigger kL4BranchPos = kOperator.mapUpDPad();

        //Algae Positions
        public static final SKTrigger kLowAlgaePos = kOperator.mapB();
        public static final SKTrigger kHighAlgaePos = kOperator.mapY();
        public static final SKTrigger kNetPos = kOperator.mapRightShoulderButton();

        //Zero Position
        public static final SKTrigger kZeroPos = kOperator.mapStart(); //Change to A or X

        //Resets
        public static final SKTrigger kResetElevatorPos = kOperator.mapSelect();  //Chnage to A or X
        public static final SKTrigger kElevatorOverride = kOperator.mapLeftJoystickPress();
        public static final SKTrigger kEndEffectorEncoderReset = kOperator.mapLeftJoystickPress();

        //End Effector Rollers
        public static final SKTrigger kIntake = kOperator.mapRightTrigger();  //TODO: consider making speeds based on axis input
        public static final SKTrigger kExtake = kOperator.mapLeftTrigger();

        //Manual Elevator and EndEffector Joystick Control
        public static final FilteredAxis kElevatorAxis = kOperator.getRawSKContorllerAxis(LEFT_Y);
        public static final FilteredAxis kEndEffectorAxis = kOperator.getRawSKContorllerAxis(RIGHT_Y);
    }

    /*
     * Defines all the ports needed to create sensors and actuators for the drivetrain.
     */
    public static class DrivePorts
    {
        // CAN IDs for the drive motors on the swerve module
        public static final CANPort kFrontLeftDriveMotorPort  = new CANPort(kFrontLeftDriveMotorID, kCANivoreName);
        public static final CANPort kFrontRightDriveMotorPort = new CANPort(kFrontRightDriveMotorID, kCANivoreName);
        public static final CANPort kBackLeftDriveMotorPort   = new CANPort(kBackLeftDriveMotorID, kCANivoreName);
        public static final CANPort kBackRightDriveMotorPort  = new CANPort(kBackRightDriveMotorID, kCANivoreName);

        // CAN IDs for the turning motors on the swerve module
        public static final CANPort kFrontLeftTurnMotorPort  = new CANPort(kFrontLeftTurnMotorID, kCANivoreName);
        public static final CANPort kFrontRightTurnMotorPort = new CANPort(kFrontRightTurnMotorID, kCANivoreName);
        public static final CANPort kBackLeftTurnMotorPort   = new CANPort(kBackLeftTurnMotorID, kCANivoreName);
        public static final CANPort kBackRightTurnMotorPort  = new CANPort(kBackRightTurnMotorID, kCANivoreName);

        // CAN IDs for the CANCoders
        public static final CANPort kFrontLeftEncoderPort  = new CANPort(kFrontLeftEncoderID, kCANivoreName);
        public static final CANPort kFrontRightEncoderPort = new CANPort(kFrontRightEncoderID, kCANivoreName);
        public static final CANPort kBackLeftEncoderPort   = new CANPort(kBackLeftEncoderID, kCANivoreName);
        public static final CANPort kBackRightEncoderPort  = new CANPort(kBackRightEncoderID, kCANivoreName);
        
        // CAN ID for IMU
        public static final CANPort kPigeonPort = new CANPort(kPigeonID, kCANivoreName);
    }
    
    public static class ClimbPorts
    {
        //The climb motor is on the drive CANbus.
        public static final CANPort kClimbMotor = new CANPort(62, kCANivoreName);
    }

    public static class ElevatorPorts
    {
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kRightElevatorMotor = new CANPort(kRightElevatorMotorID, kDefualtRioBusName);
    }

    public static class LightsPorts
    {
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kCANdle = new CANPort(kCandleID, kDefualtRioBusName);
    }

    public static class EndEffectorPorts
    {
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kEndEffectorArmMotor = new CANPort(kEndEffectorArmMotorID, kDefualtRioBusName);
        public static final CANPort kEndEffectorRollerMotor = new CANPort(kEndEffectorRollerMotorID, kDefualtRioBusName);
        public static final CANPort kLaserCanEndEffector = new CANPort(kEndEffectorLaserCanID, kDefualtRioBusName);
    }
}
