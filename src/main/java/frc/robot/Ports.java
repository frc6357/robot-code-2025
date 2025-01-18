package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Button.kBack;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftStick;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.POV;
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
import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.CANPort;
import frc.robot.utils.SKTrigger;
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
        
        // Switch modes (robot centric vs feild centric, and slow mode)
       // public final SKTrigger kRobotCentricMode = new SKTrigger(swerveController, kRightBumper.value, BUTTON);
        //public final SKTrigger kSlowMode = new SKTrigger(swerveController, kLeftBumper.value, BUTTON);

        // Reset gyro
        //public final SKTrigger kResetGyroPos = new SKTrigger(swerveController, kLeftStick.value, BUTTON);

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
        
        //ExampleButton
        public static final SKTrigger kExampleButton = new SKTrigger(kOperator, kY.value, BUTTON);

        // Party mode and Teal Lights
        public static final SKTrigger kPartyModeButton = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kLightsToTealButton = new SKTrigger(kOperator, kBack.value, BUTTON);
    }

    /**
     * Defines all the ports needed to create sensors and actuators for the drivetrain.
     */

    public static class DrivePorts
    {
        private static final String busName = "DriveCAN";

        // CAN IDs for the drive motors on the swerve module
        public static final CANPort kFrontLeftDriveMotorPort  = new CANPort(kFrontLeftDriveMotorID, busName);
        public static final CANPort kFrontRightDriveMotorPort = new CANPort(kFrontRightDriveMotorID, busName);
        public static final CANPort kRearLeftDriveMotorPort   = new CANPort(kBackLeftDriveMotorID, busName);
        public static final CANPort kRearRightDriveMotorPort  = new CANPort(kBackRightDriveMotorID, busName);

        // CAN IDs for the turning motors on the swerve module
        public static final CANPort kFrontLeftTurningMotorPort  = new CANPort(kFrontLeftTurnMotorID, busName);
        public static final CANPort kFrontRightTurningMotorPort = new CANPort(kFrontRightTurnMotorID, busName);
        public static final CANPort kRearLeftTurningMotorPort   = new CANPort(kBackLeftTurnMotorID, busName);
        public static final CANPort kRearRightTurningMotorPort  = new CANPort(kBackRightTurnMotorID, busName);

        // CAN IDs for the CANCoders
        public static final CANPort kFrontLeftTurningEncoderPort  = new CANPort(kFrontLeftEncoderID, busName);
        public static final CANPort kFrontRightTurningEncoderPort = new CANPort(kFrontRightEncoderID, busName);
        public static final CANPort kRearLeftTurningEncoderPort   = new CANPort(kBackLeftEncoderID, busName);
        public static final CANPort kRearRightTurningEncoderPort  = new CANPort(kBackRightEncoderID, busName);
        
        // CAN ID for IMU
        public static final CANPort kPigeonPort = new CANPort(kPigeonID, busName);
    }


    public static class ExamplePorts
    {
        //bus name is null
        private static final String busName = "";

        //assign a motor ID of 49 to the example motor
        public static final CANPort kExampleMotor = new CANPort(49, busName); 
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
