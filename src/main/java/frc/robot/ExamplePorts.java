package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utils.SKTrigger;
import frc.robot.utils.filters.FilteredXboxController;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
//import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.POV;

import frc.robot.utils.CANPort;
import frc.robot.utils.filters.FilteredAxis;

public class ExamplePorts
{
    public static class DriverPorts
    {
        // Driver Controller set to Xbox Controller
        public static final GenericHID kDriver = new FilteredXboxController(0).getHID();
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kTranslationXPort     = new FilteredAxis(() -> kDriver.getRawAxis(kLeftY.value));
        public static final FilteredAxis kTranslationYPort     = new FilteredAxis(() -> kDriver.getRawAxis(kLeftX.value));
        public static final FilteredAxis kVelocityOmegaPort    = new FilteredAxis(() -> kDriver.getRawAxis(kRightX.value)); 
        
        // Switch modes
        public static final SKTrigger kRobotCentricMode = new SKTrigger(kDriver, kRightBumper.value, BUTTON);
        public static final SKTrigger kSlowMode = new SKTrigger(kDriver, kLeftBumper.value, BUTTON);

        // Reset gyro
        public static final SKTrigger kResetGyroPos = new SKTrigger(kDriver, kLeftStick.value, BUTTON);

        // Party mode
        
    }
    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    
    public static class OperatorPorts
    {
        // Operator Controller set to Xbox Controller
        public static final GenericHID kOperator = new FilteredXboxController(1).getHID();

        // Example of how to use POV buttons (D-pad)
        //public static final SKTrigger kExamplePOV = new SKTrigger(kOperator, 270, POV);

        // Example of AXIS action (R/L Trigger on the controller)
        //public static final SKTrigger kExampleAXIS = new SKTrigger(kOperator, kRightTrigger.value, AXIS);

        //Example of rawAxis values (Joysticks on the controller)
        //public static final FilteredAxis kExampleRawAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        
        // Elevator
        public static final SKTrigger kTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        public static final SKTrigger kTopBranch = new SKTrigger(kOperator, kY.value, BUTTON);
        public static final SKTrigger kMiddleBranch = new SKTrigger(kOperator, kB.value, BUTTON);
        public static final SKTrigger kLowBranch = new SKTrigger(kOperator, kA.value, BUTTON);
        public static final SKTrigger kZeroPositionOperator  = new SKTrigger(kOperator, kStart.value, BUTTON);

        // Elevator Overrides
        public static final FilteredAxis kElevatorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        public static final SKTrigger kResetElevatorPos = new SKTrigger(kOperator, kBack.value, BUTTON);
        public static final SKTrigger kElevatorOverride = new SKTrigger(kOperator, kLeftStick.value, BUTTON);

    
        public static final SKTrigger armTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        public static final SKTrigger armMiddleLow = new SKTrigger(kOperator, kY.value, BUTTON);
        public static final SKTrigger armHigh = new SKTrigger(kOperator, kA.value, BUTTON);
        public static final SKTrigger intakebut = new SKTrigger(kOperator, kB.value, BUTTON);
        public static final SKTrigger zeropos = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger resetencoder = new SKTrigger(kOperator, kRightStick.value, BUTTON);
        public static final SKTrigger rollerintake = new SKTrigger(kOperator, kRightBumper.value, BUTTON);
        public static final SKTrigger rolleroutput = new SKTrigger(kOperator, kLeftBumper.value, BUTTON);

        public static final FilteredAxis endArm = new FilteredAxis(() -> kOperator.getRawAxis(kRightY.value));

    }

    /*
     * Defines all the ports needed to create sensors and actuators for the drivetrain.
     */

    public static class DrivePorts
    {
    //     private static final String busName = "DriveCAN";

    //     // CAN IDs for the drive motors on the swerve module
    //     public static final CANPort kFrontLeftDriveMotorPort  = new CANPort(13, busName);
    //     public static final CANPort kRearLeftDriveMotorPort   = new CANPort(12, busName);
    //     public static final CANPort kFrontRightDriveMotorPort = new CANPort(11, busName);
    //     public static final CANPort kRearRightDriveMotorPort  = new CANPort(10, busName);

    //     // CAN IDs for the turning motors on the swerve module
    //     public static final CANPort kFrontLeftTurningMotorPort  = new CANPort(23, busName);
    //     public static final CANPort kRearLeftTurningMotorPort   = new CANPort(22, busName);
    //     public static final CANPort kFrontRightTurningMotorPort = new CANPort(21, busName);
    //     public static final CANPort kRearRightTurningMotorPort  = new CANPort(20, busName);

    //     // CAN IDs for the CANCoders
    //     public static final CANPort kFrontLeftTurningEncoderPort  = new CANPort(33, busName);
    //     public static final CANPort kRearLeftTurningEncoderPort   = new CANPort(32, busName);
    //     public static final CANPort kFrontRightTurningEncoderPort = new CANPort(31, busName);
    //     public static final CANPort kRearRightTurningEncoderPort  = new CANPort(30, busName);
        
    //     // CAN ID for IMU
    //     public static final CANPort kPigeonPort = new CANPort(25, busName);
     }

    public static class ElevatorPorts
    {
        private static final String busName = "";
        //TODO FIX_BEFORE_TESTING - Verify CAN Bus port numbers
        public static final CANPort kRightElevatorMotor = new CANPort(0, busName);
        public static final CANPort kLeftElevatorMotor = new CANPort(0, busName);
        public static final CANPort kEncoderL = new CANPort(0, busName);
        public static final CANPort kEncoderR = new CANPort(0, busName); 
    }
    public static class EndEffectorPorts
    {
        private static final String busName = "";
        //TODO FIX_BEFORE_TESTING - Verify CAN Bus port numbers
        public static final CANPort kEndEffectorArmMotor = new CANPort(33, busName);
        public static final CANPort kEndEffectorRollerMotor = new CANPort(34, busName);
        public static final CANPort kLaserCanEndEffector = new CANPort(46, busName);

    }


    public static class ExamplePorts
    {
        //bus name is null
        private static final String busName = "";

        //assign a motor ID of 49 to the example motor
        public static final CANPort kExampleMotor = new CANPort(59, busName); 
    }
}