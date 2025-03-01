package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.MetersPerSecond;

// Unused Imports

//import static edu.wpi.first.units.Units.Rotations;
//import static edu.wpi.first.units.Units.Radians;
//import edu.wpi.first.units.measure.Distance;
//import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
//import edu.wpi.first.math.util.Units;

public final class Konstants
{
    public static final class SwerveConstants
    {
        //Device Settings and Default States
        //pigeon ID
        public static final int kPigeonID = 30; //30

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

        //The offset of the encoders in radians
        //fl 0.35
        //fr 2.225
        //bl 1.35
        //br 4.353

        public static final Double kFrontLeftEncoderOffsetRadians = (0.35); //old offset, don't delete: -0.184326171875;
        public static final Double kFrontRightEncoderOffsetRadians = (2.225); //old offset, don't delete: 0.113525390625;
        public static final Double kBackLeftEncoderOffsetRadians = (1.35); //old offset, don't delete: -0.036865234375;
        public static final Double kBackRightEncoderOffsetRadians = (4.353); //old offset, don't delete: 0.441162109375;

        //determines if the encoders are inverted
        public static final boolean kIsFrontLeftEncoderInverted = false;
        public static final boolean kIsFrontRightEncoderInverted = false;
        public static final boolean kIsBackLeftEncoderInverted = false;
        public static final boolean kIsBackRightEncoderInverted = false;

        //constants which determine if the drive motors are inverted, negative if they are inverted, positive if not.
        public static final double kFrontLeftDriveInverted = -1.0;
        public static final double kFrontRightDriveInverted = -1.0;
        public static final double kBackLeftDriveInverted = -1.0;
        public static final double kBackRightDriveInverted = -1.0;

        //if the sides are inverted
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        //if the turn motors are inverted
        public static final boolean kTurnMotorsReversed = true;

        //Robot Dimension values

        //swerve chassis width and length in inches 
        public static final int kChassisLength = 27;
        public static final int kChassisWidth = 27;

        // "Front-to-back Encoder Distance in inches"
        public static final double kFrontToBackEncoderDistInches = 21.625;
        // "Left-to-right Encoder Distance in inches"
        public static final double kLeftToRightEncoderDistInches = 21.625;

        // Module positions for kinematics, distances between encoders divided by two
        // Front left                                                                      //TODO: decide on module position in x and y or translation 2ds
        public static final double kFrontLeftXPos = kLeftToRightEncoderDistInches / 2;
        public static final double kFrontLeftYPos = kFrontToBackEncoderDistInches / 2;
        // Front right
        public static final double kFrontRightXPos = kLeftToRightEncoderDistInches / 2;
        public static final double kFrontRightYPos = -kFrontToBackEncoderDistInches / 2;
        // Back left
        public static final double kBackLeftXPos = -kLeftToRightEncoderDistInches / 2;
        public static final double kBackLeftYPos = kFrontToBackEncoderDistInches / 2;
        // Back right
        public static final double kBackRightXPos = -kLeftToRightEncoderDistInches / 2;
        public static final double kBackRightYPos = -kFrontToBackEncoderDistInches / 2;

        //radius of the wheels in inches
        public static final Double kWheelRadiusInches = 2.0;   //inches   //TODO: value in orignial spectrum code was 4 inches, was it mistaken for diameter?
        //radius of the wheels in meters. One meter is equal to 39.37 inches.
        private static final Double kWheelRadiusMeters = kWheelRadiusInches  / 39.37;   //meters
        /**Circumference of the swerve wheels for the drive conversion 
         * (circumfrance of the wheel times rotations yeilds distance travelled) */
        public static final Double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;   //meters

        // Combines a margin of error with the known wheel radius 
        // to determine an optimized value for odometry
        public static final double kWheelErrorMargin = 0.0; //inches
        public static final double kWheelRadius = ((kWheelRadiusInches + kWheelErrorMargin) / 2); //inches

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        //TODO: Tune CoupleRatio
        public static final double kCoupleGearRatio = 3.5714285714285716;

        //The gear ratios of the drive and turn motors
        //TODO: Check these values
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kTurnGearRatio = 21.428571428571427;

        //PID Constants for wheels from manual tunning
        public static final double kDriveP = 0.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.25;
        public static final double kDriveV = 0.12;
        public static final double kDriveA = 0.01;

        //rotation controller PID values
        public static final double kPRotationController = 8.0;
        public static final double kIRotationController = 0.0;
        public static final double kDRotationController = 0.2;

        //hold controller PID values
        public static final double kPHoldController = 12.0;
        public static final double kIHoldController = 0.0;
        public static final double kDHoldController = 0.0;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs kSteerGains = new Slot0Configs() //TODO - tune steering gains drive
        .withKP(100.0).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0.0);

        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        //TODO: THIS IS 2024 TUNING!!!! PLEASE UPDATE ME!!!!
        public static final Slot0Configs kDriveGains = new Slot0Configs() //TODO - tune driving gains drive
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124).withKA(0);

         // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        //Closed loop control uses feeback to control outputs of a dynamic system.
        public static final ClosedLoopOutputType kTurnClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        //Closed loop control uses feeback to control outputs of a dynamic system.
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        /**
         * The error tolerance for the PID controllers of the wheels in radians
         */
        public static final double kPIDControllerToleranceDegrees = 6.0;   //TODO: change this to radians?

        public static final double kRotationToleranceRadians = (Math.PI / 360); // rads

        //Robot Speed and Position Measurements
    
        /**The velocity limit for the swerve drive modules.*/
        public static final double kMaxVelocityMetersPerSecond = 1.0;  // m/s

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        //TODO: Tune free speed
        public static final LinearVelocity kSpeedAt12VoltsMeterPerSecond = MetersPerSecond.of(4.73);  // m/s     //TODO: find max speed in phoenix tuner x

        /**The maximum alowed angular speed of the swerve module's motors in degrees per second.*/
        public static final double kMaxModuleAngularSpeedDegreesPerSecond = 360;

        /** The max speed (m/s) the drive wheels should be allowed to go */
        public static final double kMaxDriveSpeedMetersPerSecond = 3.0;  // m/s         //TODO: Update max speed depending on robot performance
        /** The max rotation speed the turn wheels should be allowed to go */
        public static final double kMaxRotationDegreesPerSecond = 360.0;  // degrees/second

        public static final double kMaxAngularRate = 1.5 * Math.PI;
        public static final double kMaxAngularVelocity = 2 * Math.PI; // rad/s
        public static final double kMaxAngularAcceleration = Math.pow(kMaxAngularVelocity, 2); // rad/s^2

        // The inertia expereinced by the robot when attempting to drive or turn.
        //These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;

        //Starting positions of the robot when facing toward the opposing alliance's direver station. 
        public static final Rotation2d kBlueAlliancePerspective = Rotation2d.fromDegrees(0);
        public static final Rotation2d kRedAlliancePerspective = Rotation2d.fromDegrees(180);
     
        //Current Limits

        /**The current limit of the turning motors. This number should be relativley low in comparison 
        to the drive motor amperage since rotaing dosn't require nearly as much voltage as driving.*/
        public static final Current kTurningCurrentLimitAmps = Amps.of(60);

        /** Weather the current limits on the turn motors of the swerve drive are enabled. */
        public static final boolean kTurningCurrentLimitsEnabled = true;

        /**The stator current at which the wheels start to slip.
        *This needs to be tuned to your individual robot*/
        public static final double kSlipCurrentAmps = 120;            //TODO: Tune SlipCurrent

        // Simulated voltage necessary to overcome friction
        public static final double kTurnFrictionVoltage = 0.25;
        public static final double kDriveFrictionVoltage = 0.25;

        //Other Constants

        //Object containing all the neccessary coonstant values for the Swerve subsystem
        //public static final SwerveConstantsConfigurator config = new SwerveConstantsConfigurator();

        //the deadzone on the controller's joysticks
        public static final double kJoystickDeadband = 0.2;   //TODO: find approperiate deaband
        
        //CANivore name in Phoneix Tuner X for assigning CANbus names
        public static final String kCANivoreNameString = "SwerveCANivore";
        public static final CANBus kCANivoreNameCANBus = new CANBus(kCANivoreNameString);

        /** The length of the simulation loop for telemetry/logging. 
         * Affects the steering and driving inertia.*/
        public static final double kSimulationLoopPeriod = 0.005;
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
        /** Heights for the different elevator positions */
        public static enum ElevatorPosition
        {
            //TODO FIX DURING TESTING - Measure Elevator Heights

            /** Set the height to reach the top branch (L4) */ // 12.5
            kTopPosition(13.5), // 13.5 rotations of hex shaft
            /** Set the height to reach the middle branch (L3) */
            kMidPosition(9.5), // 9.5 rotations of hex shaft
            /** Set the height to reach the low branch (L2) */
            kLowPosition(7), // 7 rotations of hex shaft
            /** Set the height to reach the trough (L1) */
            kTroughPosition(3), // 3 rotations of hex shaft
            /** Set the height to reach the bottom */
            kZeroPosition(0.0);

            public final double height;

            ElevatorPosition(double height)
            {
                this.height = height;
            }
        }

        // PID Constants For Left & Right Elevator Motors (Should Be The Same)
        public static final PIDConstants leftElevator = new PIDConstants(0.07, 0.00075, 0.001);
        public static final PIDConstants rightElevator = new PIDConstants(0.07, 0.00075, 0.001);
        public static final PIDConstants balancePID = new PIDConstants(0.0, 0.0, 0.0);

        // Minimum & Maximum Integration Range For PID
        public static final double kMinInteg = 0.0;
        public static final double kMaxInteg = 0.15;

        // Positive & Negative Acceleration Limits (In %/sec)
        public static final double kPositiveAccelLimit = 2.0;
        public static final double kNegativeAccelLimit = -1.0; // Previously -5

        // Position Tolerance For The ELevator (+ or - The Target Position)
        public static final double kPositionTolerance = 0.1;

        // Minimum & Maximum Outputs For Elevator
        public static final double kElevatorMotorMinOutput = -0.5;
        public static final double kElevatorMotorMaxOutput = 0.8;

        // Maximum Current Limit For The ELevator
        public static final int kElevatorCurrentLimit = 30;
        
        /*
        Minumum & Maximum Heights The Elevator Can Be Within
        TODO Change the height and see how that works, check SmartDashboard for elevator values first.
        */
        public static final double kMaxHeight = 15;
        public static final double kMinHeight = 0;

        // Important Joystick Settings
        public static final double kJoystickChange   = 10.0;
        public static final double kJoystickDeadband = 0.2;  // Manual elevator movement axis deadband
        public static final boolean kJoystickReversed = true;  // Determines if the joystick movement is reversed
    }

    public static final class VisionConstants 
    { 
        // Each limelight has a greek letter name and an individual class for their own set of constants
        public static final class limelightAlpha {
            // Network/pipeline values
            public static final String kName = "limelight-alpha"; // Hostname? Camera name? TODO: Figure out limelight names
            public static final int kAprilTagPipeline = 0; // TODO BEFORE MERGE: check with actual AprilTag Pipeline

            // Translation (in meters) from center of robot
            public static final double kForward = 0; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0; // (x) meters right of center; negative is left
            public static final double kUp = 0; // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 0; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = false; // TODO: UPDATE LL ATTACHED TO TRUE WHEN CONNECTED
        }
        public static final class limelightBeta {
            // Network/pipeline values
            public static final String kName = "limelight-beta";
            public static final int kAprilTagPipeline = 0; // TODO BEFORE MERGE: Check with actual AprilTag Pipeline

            // Translation (in meters) from center of robot
            public static final double kForward = 0; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0; // (x) meters right of center; negative is left
            public static final double kUp = 0; // (y) meters up of center; negative is down

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 0; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]
            
            public static final boolean kAttached = false; // TODO: UPDATE LL ATTACHED TO TRUE WHEN CONNECTED
        }

        public static final double kVisionRejectDist = 1.8;

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
    public static final class EndEffectorConstants
    {
        /** Heights for the different elevator positions */
        public static enum EndEffectorPosition
        {
            /** Set the angle to reach the top branch (L4) */ // 12.5
            kTopPositionAngle(-20), // Angle

            /** Set the angle to reach the middle & low branch (L3) */
            kMidLowPositionAngle(-30), // Angle

            /** Set the angle to reach the trough (L2) */
            kTroughPositionAngle(-40), // Angle

            /** Set the height to reach the station (L1) */
            kIntakePositionAngle(-50), // Angle

            /** Set the height to reach the bottom */
            kZeroPositionAngle(-60);

            public final double angle;

            EndEffectorPosition(double angle)
            {
                this.angle = angle;
            }
        }

    //    /** Angles for the different endeffector positions */
    //    public static final double kLevel4Angle = -20;
    //    public static final double kLevel23Angle = -30;    
    //    public static final double kLevel1Angle = -40;     
    //    public static final double kIntakeAngle = -50;          
    //    public static final double kHortizontalAngle = -60;     

       /* PID values for arm motion control */
       public static final double kArmP = 1.9;
       public static final double kArmI = .0002;
       public static final double kArmD = 2.1;
       public static final double kArmV = 0.000173400381; // 1/5767

       /* Maximum motion limits for motion control */
       public static final double kArmCruiseVel = .15; // rot/sec
       public static final double kArmTargetAccel = .45; // rot/sec^2
       public static final double kArmTargetJerk = 4.5; // rot/sec^3

       /* Values for default motor speed*/
       public static final double kArmSpeed = 0.1; // rot/sec; often only used in Joystick control; Button control uses PID
       public static final double kRollerSpeed = 0.7;

       /* Current Limits */
       public static final CurrentLimitsConfigs kArmCurrentLimitsConfigs = 
        new CurrentLimitsConfigs() // Limits in Amps; time in seconds
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(100)

            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLowerLimit(50)
            .withSupplyCurrentLowerTime(0.3);

       public static final double kArmTolerance = 1;

       public static final double kCoralToLaserCanDistance = 24;

        // Important Joystick Settings
        public static final double kJoystickChange   = 0.05; // Manual setpoint value for units from 0.0 - 1.0 moved per second
        public static final double kJoystickDeadband = 0.3;  // Manual arm movement axis deadband
        public static final boolean kJoystickReversed = true;  // Determines if the joystick movement is reversed
        
        public static final double kEndEffetorMotorMinOutput = -0.5;
        public static final double kEndEffectorMotorMaxOutput = 0.8;
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

