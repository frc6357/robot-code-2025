package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.drive.GeneratedConstants;

@SuppressWarnings("unused")
public final class Konstants
{
    public static final class DriveConstants {
        public static final LinearVelocity kMaxSpeed = GeneratedConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
        public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(1.75);
        public static final LinearVelocity kMaxSpeedSLOW = kMaxSpeed.times(0.3);

        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(2); // 3/4 of a rotation per second max angular velocity
        public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2); // 1.5 rotations per second max angular velocity
        public static final AngularVelocity kMaxAngularRateSLOW = kMaxAngularRate.times(0.5); // 1/4 of a rotation per second max angular velocity

        //pigeon ID
        public static final int kPigeonID = 30; //30
    }

    /*
     * These are primarily unused and almost all of it is taken care of in TunerConstants
     */
    public static final class SwerveConstants
    {
        //Device Settings and Default States

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
        // Front left
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
        public static final Double kWheelRadiusInches = 2.0;   //inches
        //radius of the wheels in meters. One meter is equal to 39.37 inches.
        private static final Double kWheelRadiusMeters = kWheelRadiusInches  / 39.37;   //meters
        /**Circumference of the swerve wheels for the drive conversion 
         * (circumfrance of the wheel times rotations yeilds distance travelled) */
        public static final Double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;   //meters

        // Combines a margin of error with the known wheel radius 
        // to determine an optimized value for odometry
        public static final double kWheelErrorMargin = 0.0; //inches
        public static final double kWheelRadius = ((kWheelRadiusInches + kWheelErrorMargin) / 2); //inches

        //The gear ratios of the drive and turn motors
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kTurnGearRatio = 21.428571428571427;


        //PID Constants for wheels from manual tunning
        public static final double kDriveP = 0.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.25;
        public static final double kDriveV = 0.12;
        public static final double kDriveA = 0.01;

        /**
         * The error tolerance for the PID controllers of the wheels in radians
         */
        public static final double kPIDControllerToleranceDegrees = 6.0;   //TODO: change this to radians?

        public static final double kRotationToleranceRadians = (Math.PI / 360); // rads

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

        // public static final double kMaxAngularRate = 1.5 * Math.PI;
        // public static final double kMaxAngularVelocity = 2 * Math.PI; // rad/s
        // public static final double kMaxAngularAcceleration = Math.pow(kMaxAngularVelocity, 2); // rad/s^2

        // The inertia expereinced by the robot when attempting to drive or turn.
        //These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;

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
    }



    public static final class AutoConstants
    {
        // Time and speed for rollers in auto
        public static final double kIntakeAutoSpeed = 0.7;
        public static final double kExtakeAutoSpeed = -0.7;
        public static final double kIntakeAutoDurationSeconds = 0.3;  //0.5

        // PID Constants
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6.4, 0.05, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0.0);

        public static final PPHolonomicDriveController pathConfig = new PPHolonomicDriveController(kTranslationPIDConstants, kRotationPIDConstants);

        public static final PathConstraints kDefaultPathfindingConstraints = new PathConstraints(
            3.5, 3.0, 
            540, 720, 
            12, false);
    }

    public static final class SimulationRobotConstants
    {
        public static final double kPixelsPerMeter = 20;
    
        public static final double kElevatorGearing = 25; // 25:1
        public static final double kCarriageMass =
            4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
        public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
        public static final double kMinElevatorHeightMeters = 0.922; // m
        public static final double kMaxElevatorHeightMeters = 1.62; // m
    }

    public static final class ElevatorConstants
    {
        /** Heights for the different elevator positions */
        public static enum ElevatorPosition
        {
            /** Set the height to reach the top branch (L4) */ // 12.5
            kNetPosition(14), // 13.5 rotations of hex shaft
            /** Set the height to reach the top branch (L4) */ // 12.5
            kTopPosition(13.5), // 13.5 rotations of hex shaft
            /** Set the height to reach the middle branch (L3) */
            kMidPosition(9.5), // 9.5 rotations of hex shaft
            /** Set the height to reach the low branch (L2) */
            kIntakePosition(8.5), // 8.5 rotations of hex shaft
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

        public static final class CoralSubsystemConstants {
            public static final int kElevatorMotorCanId = 41;
        
            public static final class ElevatorSetpoints {
              public static final double kZero = 2; //0
              public static final double kLevel1 = 15;
              public static final double kLevel2 = 32.5;//40
              public static final double kLevel3 = 48; 
              public static final double kLevel4 = 78; //79.5
              public static final double kLowAlgae = 26;  //25 
              public static final double kHighAlgae = 38;    //37
              public static final double kNet = 78; //75
              public static final double kIntake = 30;
              public static final double kFloor = 7;
            }

            public static final class CoralSubsystem {
                public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();

                /**Max elevator speed in RPM.*/
                public static final double kMaxElevatorSpeed = 5000;
                /**Max elevator accleration in RPM / s.*/
                public static final double kMaxElevatorAcceleration = 6000;
                /** The max height of the elevator.*/
                public static final double kElevatorHeightTopLimit = 79.0;
                /** The min height of the elevator.*/
                public static final double kElevatorHeightBottomLimit = 0.0;

                /** The deadband for the elevator joystick command. */
                public static final double kManualElevatorDeadband = 0.25;
                /** The scalar value which converts joystick input to elevator speed.*/
                public static final double kManualElevatorSpeedScalar = 2.0;

                static {

                    // Configure basic settings of the elevator motor
                    elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80).voltageCompensation(12);
                    // elevatorConfig.idleMode(IdleMode.kBrake).secondaryCurrentLimit(80).voltageCompensation(12);

                    /*
                    * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
                    * will prevent any actuation of the elevator in the reverse direction if the limit switch is
                    * pressed.
                    */
                    // elevatorConfig
                    //     .limitSwitch
                    //     .reverseLimitSwitchEnabled(true)
                    //     .reverseLimitSwitchType(Type.kNormallyOpen);

                    /*
                     * Configure the closed loop controller. We want to make sure we set the
                     * feedback sensor as the primary encoder.
                    */
                    elevatorConfig
                    .inverted(true)
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(0.13)
                    //.i(0.0)
                    //.d(0.0008).dFilter(0.3)
                     .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(kMaxElevatorSpeed)
                    .maxAcceleration(kMaxElevatorAcceleration)
                    .allowedClosedLoopError(0.1);
                }
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
        public static final double kJoystickDeadband = 0.1;  // Manual elevator movement axis deadband
        public static final boolean kJoystickReversed = true;  // Determines if the joystick movement is reversed
    }

    public static final class VisionConstants { // Each limelight has a greek letter name and an individual class for their own set of constants
        public static final int kAprilTagPipeline = 0; // Default Apriltag pipeline value for all Limelights

        public static final double kLeftSideReefAlignOffset = 0.0; // Degrees of tx to align to a reef apriltag to score on its left side
        public static final double kRightSideReefAlignOffset = 0.0; // Degrees of tx to align to a reef april to score on its right side

        public static final class limelightAlpha {
            // Network/pipeline values
            public static final String kName = "limelight-alpha";

            // Translation (in meters) from center of robot
            public static final double kForward = 0.17145; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0.27305; // (x) meters right of center; negative is left
            public static final double kUp = 0.28575; // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 5; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
        }
        public static final class limelightBeta {
            // Network/pipeline values
            public static final String kName = "limelight-beta";

            // Translation (in meters) from center of robot
            public static final double kForward = 0.1651; // (z) meters forward of center; negative is backwards
            public static final double kRight = -0.276225; // (x) meters right of center; negative is left
            public static final double kUp = 0.3114625; // (y) meters up of center; negative is down

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 1.9; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = -23; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]
            
            public static final boolean kAttached = true;
        }

        public static final class AlignmentConstants {
                public static double kRotSetpoint = 0.0;

                public static double kRightYSetpoint = 0.185; //0.2
                public static double kCenterYSetpoint = 0.02;
                public static double kLeftYSetpoint = -0.1958;

                public static double kCoralXSetpoint = -0.56; //-0.54
                public static double kAlgaeXSetpoint = -0.40; //-0.45
                public static double kFarXSetpoint = -1;

                public static double kRejectDistance = 1.4; // 1.4m
        }

        public static final class PoseConstants {
            public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
            private static final Pose2d southWestLeft = new Pose2d(3.994, 5.251, new Rotation2d(-1.047));
            private static final Pose2d southWestRight = new Pose2d(3.705, 5.077, new Rotation2d(-1.047));
            private static final Pose2d southWestCenter = new Pose2d(3.856, 5.119, new Rotation2d(-1.047));

            private static Pose2d rotateLeftAroundCenter(Angle angle) {
                return southWestLeft.rotateAround(center, new Rotation2d(angle));
            }
            private static Pose2d rotateRightAroundCenter(Angle angle) {
                return southWestRight.rotateAround(center, new Rotation2d(angle));
            }
            private static Pose2d rotateCenterAroundCenter(Angle angle) {
                return southWestCenter.rotateAround(center, new Rotation2d(angle));
            }

            public static final HashMap<String, Pose2d> fieldPositions = new HashMap<String, Pose2d>() {{
              
            /* SOUTH (Face closest to driver station) */
            put("reefA", rotateLeftAroundCenter(Degrees.of(60))); // Left branch
            put("reefB", rotateRightAroundCenter(Degrees.of(60))); // Right branch
            put("reefABAlgae", rotateCenterAroundCenter(Degrees.of(60))); // Center

            /* SOUTHEAST */
            put("reefC", rotateLeftAroundCenter(Degrees.of(120)));
            put("reefD", rotateRightAroundCenter(Degrees.of(120)));
            put("reefCDAlgae", rotateCenterAroundCenter(Degrees.of(120)));

            /* NORTHEAST */
            put("reefE", rotateRightAroundCenter(Radians.of(Math.PI))); // Don't ask why this is flipped...
            put("reefF", rotateLeftAroundCenter(Radians.of(Math.PI))); // It just works
            put("reefEFAlgae", rotateCenterAroundCenter(Radians.of(Math.PI)));


            /* NORTH (Face furthest from driver station) */
            put("reefG", rotateRightAroundCenter(Degrees.of(-120)));
            put("reefH", rotateLeftAroundCenter(Degrees.of(-120)));
            put("reefGHAlgae", rotateCenterAroundCenter(Degrees.of(-120)));

            /* NORTHWEST */
            put("reefI", rotateLeftAroundCenter(Degrees.of(-60))); // new Pose2d(5.315, 5.085, new Rotation2d(-2.094))
            put("reefJ", rotateRightAroundCenter(Degrees.of(-60))); // new Pose2d(4.989, 5.261, new Rotation2d(-2.094))
            put("reefIJAlgae", rotateCenterAroundCenter(Degrees.of(-60))); // new Pose2d(5.025, 5.066, new Rotation2d(-2.094))

            /* SOUTHWEST */
            // Use this to determine all other faces
            put("reefL", new Pose2d(3.705, 5.077, new Rotation2d(-1.047))); // Don't ask why this is flipped...
            put("reefK", new Pose2d(3.994, 5.251, new Rotation2d(-1.047))); // It just works
            put("reefKLAlgae", new Pose2d(3.856, 5.119, new Rotation2d(-1.047)));
        
            /* EAST SOURCE */
            put("SourceA", new Pose2d(1.180, 6.944, new Rotation2d(2.20)));

            /* WEST SOURCE */
            put("SourceB", new Pose2d(1.237, 1.035, new Rotation2d(-2.20)));

            put("Test", new Pose2d(2.16, 3.9, new Rotation2d(0)));
        }};

        public static final HashMap<Integer, List<String>> tagDestinationMap = new HashMap<Integer, List<String>>() {{
            put(18, List.of("reefA", "reefB", "reefABAlgae")); // blue
            put(7, List.of("reefA", "reefB", "reefABAlgae")); // red
            put(17, List.of("reefC", "reefD", "reefCDAlgae")); // blue
            put(8, List.of("reefC", "reefD", "reefCDAlgae")); // red
            put(19, List.of("reefK", "reefL", "reefKLAlgae")); // blue
            put(6, List.of("reefK", "reefL", "reefKLAlgae")); // red
            put(21, List.of("reefH", "reefG", "reefGHAlgae")); // blue
            put(10, List.of("reefH", "reefG", "reefGHAlgae")); // red
            put(20, List.of("reefI", "reefJ", "reefIJAlgae")); // blue
            put(11, List.of("reefI", "reefJ", "reefIJAlgae")); // red
            put(22, List.of("reefF", "reefE", "reefEFAlgae")); // blue
            put(9, List.of("reefF", "reefE", "reefEFAlgae")); // red
        }};
        }

        public static final double kVisionRejectDist = 1.8;

    }

    /** Constants that are used when defining filters for controllers */
    public static final class OIConstants
    {
        // Controller constraints
        public static final double kDriveCoeff       = 1;
        public static final double kRotationCoeff    = 1;
        public static final double kJoystickDeadband = 0.15;
        public static final double kSlowModePercent  = 0.3;
        public static final double kSlowModeRotationPercent = 0.5;
        
        public static final double kAccelLimit = 2;

        /** The maximum elevator height in motor rotations, in which driving the robot at maximum 
         * acceleration will not cause the robot to tip over.*/
        public static final double kMaxFullSpeedElevatorHeight = 2.0;
    }
    public static final class EndEffectorConstants
    {
        /** Heights for the different elevator positions */
        public static enum EndEffectorPosition
        {
            /** Set the angle to reach the top branch (L4) */
            kTopPositionAngle(-180), //previusly -190
            /** Set the angle to reach the low branch (L2) */
            kLowPositionAngle(-175),
            /** Set the angle to reach the trough (L1) */
            kTroughPositionAngle(-140),  //-152
            /** Set the height to reach the station (Station) */
            kIntakePositionAngle(-80), 
            /** Set the height to reach the bottom */
            kZeroPositionAngle(-115), // Angle was -95, ADJUSTED WITH 2.0 AS ELEVATOR HEIGHT!
            /** Set the height to reach the net (Net) */
            kNetAngle(-80), //-90
            /** Set the height to reach the high algae (High Algae) */
            kHighAlgae(-150), 
            /** Set the height to reach the middle branch (L3) */
            kMiddleAngle(-173),
            /** Set the height to reach the low algae (Low Algae) */
            kLowAlgae(-180),
            /** Set the height to reach the floor algae (Floor) */
            kFloorAngle(-260.5),


            kIntake(-70);  //not used for station

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
       public static final double kArmP = 0.1;  //0.3
       public static final double kArmI = 0.0; //0.0002
       public static final double kArmD = 0.0; //2.1
       public static final double kArmV = 0.0; // 1/5767
       public static final double kArmFF = 0.0;

       /* Maximum motion limits for motion control */
       public static final double kArmCruiseVel = .15; // rot/sec
       public static final double kArmTargetAccel = .45; // rot/sec^2
       public static final double kArmTargetJerk = 4.5; // rot/sec^3

       /* Values for default motor speed*/
       public static final double kArmSpeed = 0.1; // rot/sec; often only used in Joystick control; Button control uses PID
       public static final double kRollerSpeed = 0.7;
       public static final double kRollerSlowSpeed = 0.50;
       public static final double kRollerSlowL2Speed = 0.45;
       public static final double kRollerSuperSpeed = 0.8;
       public static final double kRollerStop = 0;

       /* Current Limits */
       public static final CurrentLimitsConfigs kArmCurrentLimitsConfigs = 
        new CurrentLimitsConfigs() // Limits in Amps; time in seconds
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(100)

            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLowerLimit(50)
            .withSupplyCurrentLowerTime(0.3);

       public static final double kArmTolerance = 2.5; //1  //TODO tune and find tolerance

       public static final double kCoralToLaserCanDistance = 10;

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

    public static final class ClimbConstants
    {
        //Keeping P value at 0 will result in motor not spinning
        public static final double kClimbP = 1.0;         //TODO: tune climb PID
        public static final double kClimbI = 0.0;
        public static final double kClimbD = 0.0;
      //  public static final double kClimbSetpoint = 5.0;
        public static final double kKrakenSpeed = 1.0 ;  //previouslty 0.6
        public static final int kClimbCurrentLimit = 50;
        public static final double kClimbMaxPosition = 1000;
        public static final double kClimbMinPosition = -1000;
        public static final double kClimbPositionTolerance = 0.2;

        public static final Double kClimbReadyPos = -120.0;  

    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }
    
    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

