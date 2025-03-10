package frc.robot;

import static frc.robot.Konstants.kCANivoreName;
import static frc.robot.Konstants.kDefualtRioBusName;
import static frc.robot.Konstants.ElevatorConstants.kLeftElevatorMotorID;
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
import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.POV;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.LEFT_X;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.LEFT_Y;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.RIGHT_X;
import static frc.robot.utils.konstantLib.SKController.AXIS_TYPE.RIGHT_Y;
import static frc.robot.utils.konstantLib.SKController.ControllerType.XBOX;

import frc.robot.utils.konstantLib.CANPort;
import frc.robot.utils.konstantLib.SKController;
import frc.robot.utils.konstantLib.filters.FilteredAxis;
import frc.robot.utils.konstantLib.wrappers.SKTrigger;



public class Ports
{
    public static class DriverPorts 
    {
        // Driver Controller set to Xbox Controller
        //public static final CommandXboxController kDriver = new CommandXboxController(0);
        //static CommandXboxController importedKDriver = frc.robot.bindings.SK25SwerveBinder.kDriver;
        //static GenericHID kUnderlyingDriverController = importedKDriver.getHID();
        public static final GenericHID kDriver = new FilteredXboxController(0).getHID();
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftY.value));
        public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftX.value));
        public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> kDriver.getRawAxis(kRightX.value)); 

        public static final SKTrigger climbRaiseButton = new SKTrigger(kDriver, 0, POV);
        public static final SKTrigger climbLowerButton = new SKTrigger(kDriver, 180, POV);
        public static final SKTrigger climbStopButton = new SKTrigger(kDriver, 90, POV);
        public static final SKTrigger climbSlowButton = new SKTrigger(kDriver, 270, POV);
        
        // Driver Function Button (Activates secondary control scheme when held)
        public static final SKTrigger kDriveFn = new SKTrigger(kDriver, kLeftBumper.value, BUTTON);

        // Switch modes
        public static final SKTrigger kRobotCentricMode = new SKTrigger(kDriver, 180, POV); // Function Controlscheme (NOTE: This button is meant to be impossible to accidentally press)
        public static final SKTrigger kSlowMode = new SKTrigger(kDriver, kRightBumper.value, BUTTON); // Function Controlscheme

        

        // Reset gyro
        public static final SKTrigger kResetGyroPos = new SKTrigger(kDriver, kRightStick.value, BUTTON);

        // Party mode
        
       // public static final GenericHID swerveController = new FilteredXboxController(0).getHID();
        public static final SKController kDriver = new SKController(XBOX, 0);
        
        // Filtered axis (translation & rotation)
        //public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> swerveController.getRawAxis(kLeftY.value));
        // public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> swerveController.getRawAxis(kLeftX.value));
        // public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> swerveController.getRawAxis(kRightX.value)); 
        public static final FilteredAxis kTranslationXPort = kDriver.getRawSKContorllerAxis(LEFT_Y);
        public static final FilteredAxis kTranslationYPort = kDriver.getRawSKContorllerAxis(LEFT_X);
        public static final FilteredAxis kVelocityOmegaPort = kDriver.getRawSKContorllerAxis(RIGHT_X);
        
        // Driver Function Button (Activates secondary control scheme when held)
        // public static final SKTrigger kDriveFn = new SKTrigger(swerveController, kLeftBumper.value, BUTTON);
        public static final SKTrigger kDriveFn = kDriver.mapLeftShoulderButton();

        // Switch modes
        // public static final SKTrigger kRobotCentricMode = new SKTrigger(swerveController, 180, POV); // Function Controlscheme (NOTE: This button is meant to be impossible to accidentally press)
        // public static final SKTrigger kSlowMode = new SKTrigger(swerveController, kRightBumper.value, BUTTON); // Function Controlscheme
        public static final SKTrigger kRobotCentric = kDriver.mapDownDPad();
        public static final SKTrigger kSlowMode = kDriver.mapRightShoulderButton();

        // Reset gyro
        // public static final SKTrigger kResetGyroPos = new SKTrigger(swerveController, kRightStick.value, BUTTON);
        public static final SKTrigger kResetGyroPos = kDriver.mapRightJoystickPress();

    }
    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    
    public static class OperatorPorts
    {
        // Operator Controller set to Xbox Controller
        public static final GenericHID kOperator = new FilteredXboxController(1).getHID();


        /**
        * Example of how to use POV buttons (D-pad)
        * public static final SKTrigger kExamplePOV = new SKTrigger(kOperator, 270, POV);
        *
        * Example of AXIS action (R/L Trigger on the controller)
        * public static final SKTrigger kExampleAXIS = new SKTrigger(kOperator, kRightTrigger.value, AXIS);
        *
        * Example of rawAxis values (Joysticks on the controller)
        * public static final FilteredAxis kExampleRawAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        */


        // Party mode and Teal Lights
        public static final SKTrigger kPartyModeButton = new SKTrigger(kOperator, kStart.value, BUTTON);

        
        // Elevator buttons
        // Coral:
        public static final SKTrigger kIntakePos = new SKTrigger(kOperator, kLeftBumper.value, BUTTON);
        public static final SKTrigger kTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        public static final SKTrigger kLowBranch = new SKTrigger(kOperator, kA.value, BUTTON);
        // public static final SKTrigger kMiddleBranch = new SKTrigger(kOperator, kB.value, BUTTON);
        // public static final SKTrigger kTopBranch = new SKTrigger(kOperator, kY.value, BUTTON);
        // Algae:
        public static final SKTrigger kLowAlgae = new SKTrigger(kOperator, kB.value, BUTTON);
        public static final SKTrigger kHighAlgae = new SKTrigger(kOperator, kY.value, BUTTON);
        public static final SKTrigger kNetPos = new SKTrigger(kOperator, kRightBumper.value, BUTTON);

        // End Effector buttons
        // Angles:
        public static final SKTrigger kTopBranchEffector = new SKTrigger(kOperator, 0, POV);
        public static final SKTrigger kMiddleBranchEffector = new SKTrigger(kOperator, 90, POV);
        public static final SKTrigger kLowBranchEffector = new SKTrigger(kOperator, 180, POV);
        public static final SKTrigger kTroughEffector = new SKTrigger(kOperator, 270, POV);
        // Rollers:
        public static final SKTrigger kIntake = new SKTrigger(kOperator, kLeftTrigger.value, AXIS);
        public static final SKTrigger kShoot = new SKTrigger(kOperator, kRightTrigger.value, AXIS);

        // Manual Joystick Controls
        public static final FilteredAxis kElevatorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        public static final FilteredAxis kEndEffectorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kRightY.value));

        // Misc.
        public static final SKTrigger kZeroPositionOperator  = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kResetElevatorPos = new SKTrigger(kOperator, kBack.value, BUTTON);

        public static final SKTrigger kElevatorOverride = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        public static final SKTrigger resetencoder = new SKTrigger(kOperator, kRightStick.value, BUTTON);

        //public static final SKTrigger kProcessor = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        
        //Climb
        
        public static final SKTrigger rollerintake = new SKTrigger(kOperator, kRightTrigger.value, AXIS);
        public static final SKTrigger rolleroutput = new SKTrigger(kOperator, kLeftTrigger.value, AXIS);
        // public static final GenericHID kOperator = new FilteredXboxController(1).getHID();
        public static final SKController kOperator = new SKController(XBOX, 1);
        
        // Party mode and Teal Lights
        // public static final SKTrigger kPartyModeButton = new SKTrigger(kOperator, kStart.value, BUTTON);
        // public static final SKTrigger kLightsToTealButton = new SKTrigger(kOperator, kBack.value, BUTTON);
        public static final SKTrigger kPartyMode = kOperator.mapStart();
        public static final SKTrigger kLightsToTeal = kOperator.mapSelect();

        // Elevator
        // public static final SKTrigger kTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        // public static final SKTrigger kTopBranch = new SKTrigger(kOperator, kY.value, BUTTON);
        // public static final SKTrigger kMiddleBranch = new SKTrigger(kOperator, kB.value, BUTTON);
        // public static final SKTrigger kLowBranch = new SKTrigger(kOperator, kA.value, BUTTON);
        // public static final SKTrigger kZeroPositionOperator  = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kTrough = kOperator.mapX();
        public static final SKTrigger kTopBranch = kOperator.mapY();
        public static final SKTrigger kMiddleBranch = kOperator.mapB();
        public static final SKTrigger kLowBranch = kOperator.mapA();
        public static final SKTrigger kZeroPositionOperator = kOperator.mapStart();

        // Elevator Overrides
        // public static final FilteredAxis kElevatorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        // public static final SKTrigger kResetElevatorPos = new SKTrigger(kOperator, kBack.value, BUTTON);
        // public static final SKTrigger kElevatorOverride = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        public static final FilteredAxis kElevatorAxis = kOperator.getRawSKContorllerAxis(LEFT_Y);
        public static final SKTrigger kResetElevatorPos = kOperator.mapSelect();
        public static final SKTrigger kElevatorOverride = kOperator.mapLeftJoystickPress();

    
        // public static final SKTrigger armTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        // public static final SKTrigger armMiddleLow = new SKTrigger(kOperator, kY.value, BUTTON);
        // public static final SKTrigger armHigh = new SKTrigger(kOperator, kA.value, BUTTON);
        // public static final SKTrigger intakebut = new SKTrigger(kOperator, kB.value, BUTTON);
        // public static final SKTrigger zeropos = new SKTrigger(kOperator, kStart.value, BUTTON);
        // public static final SKTrigger resetencoder = new SKTrigger(kOperator, kRightStick.value, BUTTON);
        // public static final SKTrigger rollerintake = new SKTrigger(kOperator, kRightBumper.value, BUTTON);
        // public static final SKTrigger rolleroutput = new SKTrigger(kOperator, kLeftBumper.value, BUTTON);
        public static final SKTrigger kArmTrough = kOperator.mapX();
        public static final SKTrigger kArmMiddleLow = kOperator.mapY();
        public static final SKTrigger kArmHigh = kOperator.mapA();
        public static final SKTrigger kIntakeBut = kOperator.mapB();
        public static final SKTrigger kZeroPos = kOperator.mapStart();
        public static final SKTrigger kResetEncoder = kOperator.mapRightJoystickPress();
        public static final SKTrigger kRollerIntake = kOperator.mapRightShoulderButton();
        public static final SKTrigger kRollerOutput = kOperator.mapLeftShoulderButton();

        // public static final FilteredAxis endArm = new FilteredAxis(() -> kOperator.getRawAxis(kRightY.value));
        public static final FilteredAxis kEndArm = kOperator.getRawSKContorllerAxis(RIGHT_Y);
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
        private static final String busName = kCANivoreNameString;
        public static final CANPort kClimbMotor = new CANPort(62, busName);
        
    }

    public static class ElevatorPorts
    {
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kRightElevatorMotor = new CANPort(41, kDefualtRioBusName);
    }

    public static class LightsPorts
    {
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kCANdle = new CANPort(kCandleID, kDefualtRioBusName);
    }

    public static class EndEffectorPorts
    {
        //TODO FIX_BEFORE_TESTING - Verify CAN Bus port numbers
        //The bus name is empty, because this subsystem does not use a named CANbus.
        public static final CANPort kEndEffectorArmMotor = new CANPort(kEndEffectorArmMotorID, kDefualtRioBusName);
        public static final CANPort kEndEffectorRollerMotor = new CANPort(kEndEffectorRollerMotorID, kDefualtRioBusName);
        public static final CANPort kLaserCanEndEffector = new CANPort(kEndEffectorLaserCanID, kDefualtRioBusName);

    }
}
