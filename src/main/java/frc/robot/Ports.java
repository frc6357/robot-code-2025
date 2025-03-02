package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kCANivoreNameString;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kPigeonID;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
//import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.POV;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utils.CANPort;
import frc.robot.utils.SKTrigger;
import frc.robot.utils.filters.FilteredAxis;
import frc.robot.utils.filters.FilteredXboxController;

public class Ports
{
    public static class DriverPorts
    {
        // Driver Controller set to Xbox Controller
        //public static final CommandXboxController kDriver = new CommandXboxController(0);
        //static CommandXboxController importedKDriver = frc.robot.bindings.SK25SwerveBinder.kDriver;
        //static GenericHID kUnderlyingDriverController = importedKDriver.getHID();
        public static final GenericHID swerveController = new FilteredXboxController(0).getHID();
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> swerveController.getRawAxis(kLeftY.value));
        public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> swerveController.getRawAxis(kLeftX.value));
        public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> swerveController.getRawAxis(kRightX.value)); 
        
        // Driver Function Button (Activates secondary control scheme when held)
        public static final SKTrigger kDriveFn = new SKTrigger(swerveController, kLeftBumper.value, BUTTON);

        // Switch modes
        public static final SKTrigger kRobotCentricMode = new SKTrigger(swerveController, 180, POV); // Function Controlscheme (NOTE: This button is meant to be impossible to accidentally press)
        public static final SKTrigger kSlowMode = new SKTrigger(swerveController, kRightTrigger.value, BUTTON); // Function Controlscheme

        // Reset gyro
        public static final SKTrigger kResetGyroPos = new SKTrigger(swerveController, kRightStick.value, BUTTON);

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
        

        // Party mode and Teal Lights
        public static final SKTrigger kPartyModeButton = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kLightsToTealButton = new SKTrigger(kOperator, kBack.value, BUTTON);
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

        // Vision Commands
        public static final SKTrigger kReefAlignCommand = new SKTrigger(kOperator, kLeftBumper.value, BUTTON);

        // Party mode
        public static final SKTrigger kPartyMode = new SKTrigger(kOperator, kRightBumper.value, BUTTON);
        public static final SKTrigger kLightsOff = new SKTrigger(kOperator, 90, POV);
    }

    /*
     * Defines all the ports needed to create sensors and actuators for the drivetrain.
     */

    public static class DrivePorts
    {
        // CAN IDs for the drive motors on the swerve module
        public static final CANPort kFrontLeftDriveMotorPort  = new CANPort(kFrontLeftDriveMotorID, kCANivoreNameString);
        public static final CANPort kFrontRightDriveMotorPort = new CANPort(kFrontRightDriveMotorID, kCANivoreNameString);
        public static final CANPort kBackLeftDriveMotorPort   = new CANPort(kBackLeftDriveMotorID, kCANivoreNameString);
        public static final CANPort kBackRightDriveMotorPort  = new CANPort(kBackRightDriveMotorID, kCANivoreNameString);

        // CAN IDs for the turning motors on the swerve module
        public static final CANPort kFrontLeftTurnMotorPort  = new CANPort(kFrontLeftTurnMotorID, kCANivoreNameString);
        public static final CANPort kFrontRightTurnMotorPort = new CANPort(kFrontRightTurnMotorID, kCANivoreNameString);
        public static final CANPort kBackLeftTurnMotorPort   = new CANPort(kBackLeftTurnMotorID, kCANivoreNameString);
        public static final CANPort kBackRightTurnMotorPort  = new CANPort(kBackRightTurnMotorID, kCANivoreNameString);

        // CAN IDs for the CANCoders
        public static final CANPort kFrontLeftEncoderPort  = new CANPort(kFrontLeftEncoderID, kCANivoreNameString);
        public static final CANPort kFrontRightEncoderPort = new CANPort(kFrontRightEncoderID, kCANivoreNameString);
        public static final CANPort kBackLeftEncoderPort   = new CANPort(kBackLeftEncoderID, kCANivoreNameString);
        public static final CANPort kBackRightEncoderPort  = new CANPort(kBackRightEncoderID, kCANivoreNameString);
        
        // CAN ID for IMU
        public static final CANPort kPigeonPort = new CANPort(kPigeonID, kCANivoreNameString);
    }

    public static class ElevatorPorts
    {
        private static final String busName = "";
        //TODO FIX_BEFORE_TESTING - Verify CAN Bus port numbers
        public static final CANPort kRightElevatorMotor = new CANPort(40, busName);
        public static final CANPort kLeftElevatorMotor = new CANPort(41, busName);
        //public static final CANPort kEncoderL = new CANPort(43, busName);
        //public static final CANPort kEncoderR = new CANPort(44, busName); 
    }


    public static class ExamplePorts
    {
        //bus name is null
        private static final String busName = "";

        //assign a motor ID of 49 to the example motor
        public static final CANPort kExampleMotor = new CANPort(59, busName); 
    }

    public static class LightsPorts
    {
        //bus name is null
        private static final String busName = "";
        //assign an ID of 48 to the CANdle
        public static final CANPort kCANdle = new CANPort(48, busName);
    }

    //
}
