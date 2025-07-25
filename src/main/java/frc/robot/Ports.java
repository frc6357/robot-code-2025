package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Konstants.SwerveConstants.kBackLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kBackRightTurnMotorID;
import static frc.robot.Konstants.kCANivoreName;
import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightDriveMotorID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderID;
import static frc.robot.Konstants.SwerveConstants.kFrontRightTurnMotorID;
import static frc.robot.Konstants.SwerveConstants.kPigeonID;
//import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.POV;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.utils.CANPort;
import frc.robot.utils.SKTrigger;
import frc.robot.utils.filters.DeadbandFilter;
import frc.robot.utils.filters.FilteredAxis;
import frc.robot.utils.filters.FilteredXboxController;

// Unused Imports

//import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
//import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;

public class Ports
{
    public static class DriverPorts
    {
        // Driver Controller set to Xbox Controller
        //public static final CommandXboxController kDriver = new CommandXboxController(0);
        //static CommandXboxController importedKDriver = frc.robot.bindings.SK25SwerveBinder.kDriver;
        //static GenericHID kUnderlyingDriverController = importedKDriver.getHID();
        public static GenericHID kDriver = new FilteredXboxController(0).getHID();
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kTranslationXPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftY.value));
        public static final FilteredAxis kTranslationYPort = new FilteredAxis(() -> kDriver.getRawAxis(kLeftX.value));
        public static final FilteredAxis kVelocityOmegaPort = new FilteredAxis(() -> kDriver.getRawAxis(kRightX.value)); 
        
        public static final SKTrigger climbRaiseButton = new SKTrigger(kDriver, 270, POV);
        public static final SKTrigger climbLowerButton = new SKTrigger(kDriver, 90, POV);
        // public static final SKTrigger climbStopButton = new SKTrigger(kDriver, 90, POV);
        // public static final SKTrigger climbSlowButton = new SKTrigger(kDriver, 270, POV);
        
        // Vision subsystem enable/disable
        public static final SKTrigger kVisionOff = new SKTrigger(kDriver, 180, POV);
        public static final SKTrigger kVisionOn = new SKTrigger(kDriver, 0, POV);
        
        // Vision Driving buttons
        public static final SKTrigger kResetPoseToVision = new SKTrigger(kDriver, kB.value, BUTTON);
        public static final SKTrigger kForceResetPoseToVision = new SKTrigger(kDriver, kY.value, BUTTON);
        public static final SKTrigger kAlignToReef = new SKTrigger(kDriver, kX.value, BUTTON);
        public static final SKTrigger kLeftReef = new SKTrigger(kDriver, kLeftTrigger.value, AXIS);
        public static final SKTrigger kRightReef = new SKTrigger(kDriver, kRightTrigger.value, AXIS);
        
        // Driver Function Button (Activates secondary control scheme when held)
        public static final SKTrigger kDriveFn = new SKTrigger(kDriver, kRightBumper.value, BUTTON);
        
        // Switch modes
        public static final SKTrigger kRobotCentricMode = new SKTrigger(kDriver, kRightBumper.value, BUTTON); // Function Controlscheme (NOTE: This button is meant to be impossible to accidentally press)
        public static final SKTrigger kSlowMode = new SKTrigger(kDriver, kLeftBumper.value, BUTTON); // Function Controlscheme
        
        // Reset gyro
        public static final SKTrigger kResetGyroPos = new SKTrigger(kDriver, kRightStick.value, BUTTON);
        
        // Controller swap
        public static final SKTrigger kDSwapStart = new SKTrigger(kDriver, kStart.value, BUTTON);
        public static final SKTrigger kDSwapBack = new SKTrigger(kDriver, kBack.value, BUTTON);
        
        public static void swapDriverController() {
            kDriver.setRumble(RumbleType.kBothRumble, 0.5);
            if(kDriver.getPort() == 0) {
                kDriver = new FilteredXboxController(1).getHID();
            }
            else {
                kDriver = new FilteredXboxController(0).getHID();
            }
        }

        public static void killDriverRumble() {
            kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }
    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    
    public static class OperatorPorts
    {
        // Operator Controller set to Xbox Controller
        public static GenericHID kOperator = new FilteredXboxController(1).getHID();
        
        
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
        //public static final SKTrigger kTrough = new SKTrigger(kOperator, kX.value, BUTTON);
        //public static final SKTrigger kLowBranch = new SKTrigger(kOperator, kA.value, BUTTON);
        // public static final SKTrigger kMiddleBranch = new SKTrigger(kOperator, kB.value, BUTTON);
        // public static final SKTrigger kTopBranch = new SKTrigger(kOperator, kY.value, BUTTON);
        // Algae:
        public static final SKTrigger kFloorAlgae = new SKTrigger(kOperator, kA.value, BUTTON);
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
        public static final SKTrigger kIntake = new SKTrigger(kOperator, kRightTrigger.value, AXIS);
        public static final SKTrigger kShoot = new SKTrigger(kOperator, kLeftTrigger.value, AXIS);

        // Manual Joystick Controls
        public static final FilteredAxis kElevatorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value), new DeadbandFilter(kJoystickDeadband));
        public static final FilteredAxis kEndEffectorAxis = new FilteredAxis(() -> kOperator.getRawAxis(kRightY.value), new DeadbandFilter(kJoystickDeadband));

        // Misc.
        public static final SKTrigger kZeroPositionOperator  = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kResetElevatorPos = new SKTrigger(kOperator, kBack.value, BUTTON);

        public static final SKTrigger kElevatorOverride = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        public static final SKTrigger resetencoder = new SKTrigger(kOperator, kRightStick.value, BUTTON);

        //public static final SKTrigger kProcessor = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        
        // Controller swap
        public static final SKTrigger kOSwapStart = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kOSwapBack = new SKTrigger(kOperator, kBack.value, BUTTON);

        public static void swapOperatorController() {
            kOperator.setRumble(RumbleType.kBothRumble, 0.5);
            if(kOperator.getPort() == 1) {
                kOperator = new FilteredXboxController(0).getHID();
            }
            else {
                kOperator = new FilteredXboxController(1).getHID();
            }
        }

        public static void killOperatorRumble() {
            kOperator.setRumble(RumbleType.kBothRumble, 0.0);
        }
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
        private static final String busName = kCANivoreName;
        public static final CANPort kClimbMotor = new CANPort(62, busName);
        
    }

    public static class ElevatorPorts
    {
        private static final String busName = "";
        public static final CANPort kRightElevatorMotor = new CANPort(41, busName);
        //public static final CANPort kLeftElevatorMotor = new CANPort(42, busName);
    }

    public static class LightsPorts
    {
    private static final String busName = "";
    //assign an ID of 48 to the CANdle
    public static final CANPort kCANdle = new CANPort(48, busName);
    }

    public static class EndEffectorPorts
    {
        private static final String busName = "";
        //assign an ID of 48 to the CANdle
        public static final CANPort kCANdle = new CANPort(48, busName);
    
        public static final CANPort kEndEffectorArmMotor = new CANPort(33, busName);
        public static final CANPort kEndEffectorRollerMotor = new CANPort(34, busName);
        public static final CANPort kLaserCanEndEffector = new CANPort(46, busName);

    }


    // public static class ExamplePorts
    // {
    //     //bus name is null
    //     private static final String busName = "";

    //     //assign a motor ID of 49 to the example motor
    //     public static final CANPort kExampleMotor = new CANPort(59, busName); 
    // }
}
