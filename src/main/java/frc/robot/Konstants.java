package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


//import edu.wpi.first.units.measure.Distance;

public final class Konstants
{

    public static final class SwerveConstants
    {
        //swerve motor IDs
        public static final int kFrontLeftDriveMotorID = 1; //1
        public static final int kFrontRightDriveMotorID = 2; //2
        public static final int kBackLeftDriveMotorID = 3; //3
        public static final int kBackRightDriveMotorID = 4; //4

        public static final int kFrontLeftTurnMotorID = 11; //11
        public static final int kFrontRightTurnMotorID = 12; //12
        public static final int kBackLeftTurnMotorID = 13; //13
        public static final int kBackRightTurnMotorID = 14; //14

        //encoder IDs
        public static final int kFrontLeftEncoderID = 21; //21
        public static final int kFrontRightEncoderID = 22; //22
        public static final int kBackLeftEncoderID = 23; //23
        public static final int kBackRightEncoderID = 24; //24

        //swerve chassis width and length in inches 
        public static final int kChassisLength = 27;
        public static final int kChassisWidth = 27;

        //double versions
        public static final Double kFrontLeftEncoderOffsetRadians = 0.40283203125;
        public static final Double kFrontRightEncoderOffsetRadians = -0.044677734375;
        public static final Double kBackLeftEncoderOffsetRadians = -0.21875;
        public static final Double kBackRightEncoderOffsetRadians = -0.08642578125;
        //TODO: go to constants and convert angle values to doubles
        
        //PID Constants for wheels from manual tunning
        public static final double kDriveP = 0.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        /**
         * The error tolerance for the PID controllers of the wheels in radians
         */
        public static final double kPIDControllerToleranceDegrees = 6.0;
        
        //pigeon ID
        public static final int kPigeonID = 30; //30

        //constants which determine if the motors are inverted, negative if they are inverted, positive if not.
        public static final double kFrontRightInverted = -1.0;
        public static final double kFrontLeftInverted = -1.0;
        public static final double kBackRightInverted = -1.0;
        public static final double kBackLeftInverted = -1.0;

        //the deadzone on the controller's joysticks
        public static final double kJoystickDeadzone = 0.2;

        //the velocity limit for the swerve drive modules
        public static final double kMaxVelocityMetersPerSecond = 1.0;

        //radius of the wheels in inches
        private static final Double kWheelRadiusInches = 2.0;   //inches
        //radius of the wheels in meters
        private static final Double kWheelRadiusMeters = kWheelRadiusInches  / 39.37;   //meters
        //circumference of the swerve wheels for the drive conversion (circumfrance of the wheel times rotations yeilds distance travelled)
        public static final Double kWheelCircumference = 2 * Math.PI * kWheelRadiusMeters;   //meters

        //CANivore name in Phoneix Tuner X for assigning CANbus names
        public static final String kCANivoreName = "SwerveCANivore";



        //experimental swerve constants
        public static final double kDeadband = 0.3; //TODO: Update based on driver preference

        //8 swerve motor IDs for Ports
        // Front left            //TODO: use actual IDs from above and change them
        public static final int kFrontLeftDriveMotorId = 13;
        public static final int kFrontLeftSteerMotorId = 23;
        // Front right
        public static final int kFrontRightDriveMotorId = 11;
        public static final int kFrontRightSteerMotorId = 21;
        // Back left
        public static final int kBackLeftDriveMotorId = 12;
        public static final int kBackLeftSteerMotorId = 22;
        // Back right
        public static final int kBackRightDriveMotorId = 10;
        public static final int kBackRightSteerMotorId = 20;

        // Encoder IDs and offsets:
        // Front left
        public static final int kFrontLeftEncoderId = 33;
        public static final double kFrontLeftEncoderOffset = 0.19482421875;

        // Front right
        public static final int kFrontRightEncoderId = 31;
        public static final double kFrontRightEncoderOffset = 0.141845703125;

        // Back left
        public static final int kBackLeftEncoderId = 32;
        public static final double kBackLeftEncoderOffset = -0.277099609375;
        
        // Back right
        public static final int kBackRightEncoderId = 30;
        public static final double kBackRightEncoderOffset = -0.087646484375;

        // "Front-to-back Encoder Distance in inches"
        public static final double kFTBEncoderDistInches = 21.6875;
        //"Left-to-right Encoder Distance in inches"
        public static final double kLTREncoderDistInches = 21.59375;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs kSteerGains = new Slot0Configs() //TODO - tune steering gains drive
        .withKP(4.8).withKI(0).withKD(0.1)
        .withKS(0.25).withKV(0.12).withKA(0.01);

        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        //TODO: THIS IS 2024 TUNING!!!! PLEASE UPDATE ME!!!!
        public static final Slot0Configs kDriveGains = new Slot0Configs() //TODO - tune driving gains drive
        .withKP(2.3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
        
        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        //TODO: Tune SlipCurrent
        public static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        //TODO: Tune free speed
        public static final double kSpeedAt12VoltsMps = 4.73;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        //TODO: Tune CoupleRatio
        public static final double kCoupleRatio = 3.5714285714285716;

        //TODO: Check these values
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kSteerGearRatio = 21.428571428571427;

        public static final boolean kInvertLeftSide = false;
        public static final boolean kSteerMotorReversed = true;
        public static final boolean kInvertRightSide = true;

        public static final String kCANbusName = "SwerveCANivore";
        public static final int kPigeonId = 30;

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;


        // Combines a margin of error with the known wheel radius 
        // to determine an optimized value for odometry
        public static final double kWheelMOE= 0.0;
        public static final double kWheelRadius = (4.0 + kWheelMOE / 2);

        // These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        public static final double kSteerFrictionVoltage = 0.25;
        public static final double kDriveFrictionVoltage = 0.25;

        // Module positions for kinematics
        // Front left
        public static final double kFrontLeftXPos = kLTREncoderDistInches / 2;
        public static final double kFrontLeftYPos = kFTBEncoderDistInches / 2;
        // Front right
        public static final double kFrontRightXPos = kLTREncoderDistInches / 2;
        public static final double kFrontRightYPos = -kFTBEncoderDistInches / 2;
        // Back left
        public static final double kBackLeftXPos = -kLTREncoderDistInches / 2;
        public static final double kBackLeftYPos = kFTBEncoderDistInches / 2;
        // Back right
        public static final double kBackRightXPos = -kLTREncoderDistInches / 2;
        public static final double kBackRightYPos = -kFTBEncoderDistInches / 2;

        public static final double kMaxModuleAngularSpeedDegreesPerSecond = 360;

        /** The max speed the drive wheels should be allowed to go */
        public static final double kMaxSpeedMetersPerSecond = 3.0; //TODO: Update max speed depending on robot performance
        public static final double kMaxRotationDegreesPerSecond = 360.0;
    }

    public static final class AutoConstants
    {
        public static List<String> autoList = new ArrayList<String>(Arrays.asList("P4_Taxi"));

        // Autonomous translation constraints
        public static final double          kMaxSpeedMetersPerSecond               = 3;
        public static final double          kMaxAccelerationMetersPerSecondSquared = 2;
        // public static final PathConstraints kPathConstraints                       =
        //         new PathConstraints(kMaxSpeedMetersPerSecond,
        //             kMaxAccelerationMetersPerSecondSquared);

        // public static final PathConstraints kFastConstraints =
        //     new PathConstraints(4.5, 3.5);

        // PID Constants
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6, 0, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0);
        public static final PPHolonomicDriveController kAutoPathConfig = new PPHolonomicDriveController(
            kTranslationPIDConstants,
            kRotationPIDConstants,
            Robot.kDefaultPeriod
            //kMaxSpeedMetersPerSecond,
            //Math.hypot(SwerveConstants.kFTBEncoderDistInches / 2, SwerveConstants.kLTREncoderDistInches / 2), 
            //new ReplanningConfig()
        );

    }

    public static final class ElevatorConstants
    {
        public static final PIDConstants rightElevator = new PIDConstants(0.01, 0.0, 0.0);
        public static final PIDConstants leftElevator = new PIDConstants(0.01, 0.0, 0.0);
        public static final PIDConstants balancePID = new PIDConstants(0.0, 0.0, 0.0);

        public static final double kElevatorBalanceTolerance = 5.0;
        public static final double spoolDiameter = 0.75; //Inches
        public static final double gearRatio = 1.0; //Shaft rotations / 1 motor rotation
        public static final double elevatorHeight = 11.0; //Inches

        public static final double elevatorConversion = 1.0 / 87.0; //inches moved per motor rotation
        public static final double kPositionTolerance = 2.0;
        public static final double kElevatorMotorMinOutput = -1.0;
        public static final double kElevatorMotorMaxOutput = 1.0;

        public static final double kElevatorUpSpeed = 1.0;
        public static final double kElevatorDownSpeed = 1.0;

        //public static final double kElevatorUpSpeedLeft = -1.0;
        //public static final double kElevatorDownSpeedLeft = 1.0;

        public static final double kMin = 0.0;
        public static final double kMax = 1.0;
    
        public static final double kJoystickChange   = 0.1; // Manual setpoint value for units from 0.0 - 1.0 moved per second
        public static final double kJoystickDeadband = 0.3;  // Manual arm movement axis deadband

        public static final boolean kJoystickReversed = true;  // Determines if the joystick movement is reversed
    }

        /** Constants that are used when defining filters for controllers */
        public static final class OIConstants
        {
            // Controller constraints
            public static final double kDriveCoeff       = 0.95;
            public static final double kRotationCoeff    = 0.95;
            public static final double kJoystickDeadband = 0.15;
            public static final double kSlowModePercent  = 0.2;
            
            public static final double kAccelLimit = 2;
        }

    public static final class LightConstants
    {
        public static final int numLedOnBot = 240;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;
    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }
    
    

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

