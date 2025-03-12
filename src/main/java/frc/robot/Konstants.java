package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
//import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
import frc.robot.preferences.SKPreferences;

//import static edu.wpi.first.units.Units.Rotations;
//import static edu.wpi.first.units.Units.Radians;
//import edu.wpi.first.units.measure.Distance;
//import frc.robot.subsystems.swerve.SwerveConstantsConfigurator;
//import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SKSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

@SuppressWarnings("unused")
public final class Konstants
{

    // Generated by the Tuner X Swerve Project Generator
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(70.0) // Amps
                .withSupplyCurrentLowerLimit(50) // Amps
                .withSupplyCurrentLowerTime(0.5) // Seconds
                .withSupplyCurrentLimitEnable(true)

                .withStatorCurrentLimit(60) // Amps
                .withStatorCurrentLimitEnable(true)
            );

        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("SwerveCANivore", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = true;
        private static final boolean kInvertRightSide = false;

        private static final int kPigeonId = 30;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 11;
        private static final int kFrontLeftEncoderId = 21;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.184326171875);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(10.8125);
        private static final Distance kFrontLeftYPos = Inches.of(10.8125);

        // Front Right
        private static final int kFrontRightDriveMotorId = 2;
        private static final int kFrontRightSteerMotorId = 12;
        private static final int kFrontRightEncoderId = 22;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.113525390625);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(10.8125);
        private static final Distance kFrontRightYPos = Inches.of(-10.8125);

        // Back Left
        private static final int kBackLeftDriveMotorId = 3;
        private static final int kBackLeftSteerMotorId = 13;
        private static final int kBackLeftEncoderId = 23;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.036865234375);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-10.8125);
        private static final Distance kBackLeftYPos = Inches.of(10.8125);

        // Back Right
        private static final int kBackRightDriveMotorId = 4;
        private static final int kBackRightSteerMotorId = 14;
        private static final int kBackRightEncoderId = 24;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.441162109375);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-10.8125);
        private static final Distance kBackRightYPos = Inches.of(-10.8125);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        /**
         * Creates a CommandSwerveDrivetrain instance.
         * This should only be called once in your robot program,.
         */
        public static SKSwerve createDrivetrain() {
            return new SKSwerve(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
            );
        }


        /**
         * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
         */
        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
             * @param modules               Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency The frequency to run the odometry loop. If
             *                                unspecified or set to 0 Hz, this is 250 Hz on
             *                                CAN FD, and 100 Hz on CAN 2.0.
             * @param modules                 Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
             *                                  unspecified or set to 0 Hz, this is 250 Hz on
             *                                  CAN FD, and 100 Hz on CAN 2.0.
             * @param odometryStandardDeviation The standard deviation for odometry calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param visionStandardDeviation   The standard deviation for vision calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param modules                   Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
                );
            }
        }
    }   




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

        /**Swerve chassis width and length in inches*/ 
        public static final int kChassisLength = 27;

        /**The deadzone on the controller's joysticks*/
        public static final double kJoystickDeadband = 0.2;

        /** The percentage of speed to drive in slow mode. Currently 30% of defualt speed.*/
        public static final double kSlowModePercentage = 0.3;
    }

    /** Defines constraints and information for autonomous development */
    public static final class AutoConstants
    {
        // Time and speed for rollers in auto
        public static final double kIntakeAutoSpeed = -0.7;
        public static final double kExtakeAutoSpeed = 0.7;
        public static final double kIntakeAutoDurationSeconds = 0.5;
        public static final double kExtakeAutoDurationSeconds = 0.5;

        // PID Constants for translation and rotatio PID controller of swerve for pathplanner
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6, 0, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0);

        public static final PPHolonomicDriveController pathConfig = new PPHolonomicDriveController(kTranslationPIDConstants, kRotationPIDConstants);
    }

    public static final class ElevatorConstants
    {
        //IDs
        public static final int kRightElevatorMotorID = 41;
        public static final int kLeftElevatorMotorID = 42;

        public static final class ElevatorSetpoints
        {
            //The elevator setpoint heights in motor rotations
            public static final double kZero = 0;
            public static final double kTrough = 15;
            public static final double kLevel2 = 40;
            public static final double kLevel3 = 56; //-215
            public static final double kLevel4 = 79.5; //-190
            public static final double kLowAlgae = 22;  //-173
            public static final double kHighAlgae = 40;  //-173
            public static final double kNet = 75;
            public static final double kIntake = 30;
        }

        //Define elevato config object
        public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();

        static {

            // Configure basic settings of the elevator motor
            elevatorConfig.idleMode(IdleMode.kCoast)  //Coast mdoe
                .smartCurrentLimit(50)  //stall current limit
                .voltageCompensation(12);  //voltage compensation   //TODO: check voltage comp when tuning elevator PID

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
            .p(0.05)
            .i(0.0)
            .d(0.0008).dFilter(0.3)
                .outputRange(-0.45, 0.5)
            .maxMotion
            // Set MAXMotion parameters for position control
            .maxVelocity(2500)
            .maxAcceleration(6000)
            .allowedClosedLoopError(0.05);
        }

        // Joystick Settings
        public static final double kJoystickChange   = 10.0;
        public static final double kJoystickDeadband = 0.1;  // Manual elevator movement axis deadband
        public static final boolean kJoystickReversed = true;  // Determines if the joystick movement is reversed
    }

    public static final class VisionConstants 
    { 
        // Each limelight has a greek letter name and an individual class for their own set of constants
        public static final class limelightAlpha {
            // Network/pipeline values
            public static final String kName = "limelight-alpha"; // Hostname? Camera name?
            public static final int kAprilTagPipeline = 0;

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
            public static final int kAprilTagPipeline = 0; 

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
        public static final double kSlowModePercent  = 0.05;
        
        public static final double kAccelLimit = 2;

        /** The maximum elevator height in motor rotations, in which driving the robot at maximum 
         * acceleration will not cause the robot to tip over.*/
        public static final double kMaxFullSpeedElevatorHeight = 2.0;
    }
    public static final class EndEffectorConstants
    {

        //IDs
        public static final int kEndEffectorArmMotorID = 33;
        public static final int kEndEffectorRollerMotorID = 35;
        public static final int kEndEffectorLaserCanID = 42;

        /** Heights for the different elevator positions in degrees */
        public static enum EndEffectorPosition
        {
            /** Set the angle to reach the top branch (L4) */
            kL4Angle(-190),
            /** Set the angle to reach the middle branch (L3) */
            kMiddleAngle(-215),
            /** Set the angle to reach the low branch (L2) */
            kL2Angle(-215),
            /** Set the angle to reach the trough (L1) */
            kTroughAngle(-195),
            /** Set the height to reach the station */
            kStationAngle(-100), //TODO: Angle -100 or -70 (check this)
            /** Set the height to reach the bottom */
            kZeroPositionAngle(-60),
            /** Set the angle to reach the net */
            kNetAngle(-90),
            /** Set the angle to reach the high algae on the reef */
            kHighAlgae(-173),
            /** Set the angle to reach the low algae on the reef */
            kLowAlgae(-173);

            public final double angle;

            EndEffectorPosition(double angle)
            {
                this.angle = angle;
            }
        }    

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
       /** Intake speed, where the negative version of this number intakes and the positive version extakes. */
       public static final double kRollerSpeed = 0.7;
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
        public static final int kCandleID = 48;

        public static final int numLedOnBot = 240;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;
    }

    public static final class ClimbConstants
    {
        //Keeping P value at 0 will result in motor not spinning
        public static final double kClimbP = 1.0;
        public static final double kClimbI = 0.0;
        public static final double kClimbD = 0.0;
        //public static final double kClimbSetpoint = 5.0;
        public static final double kKrakenSpeed = .3 ;
        public static final int kClimbCurrentLimit = 50;
        public static final double kClimbMaxPosition = 120;
        public static final double kClimbMinPosition = 0.0;
        public static final double kClimbPositionTolerance = 0.2;

    }
      
    /**CANivore name in Phoneix Tuner X for assigning CANbus names*/
    public static final String kCANivoreName = "SwerveCANivore";

    /** The name of the CANbus when using the rio. <""> and <"rio"> are the two working rio  bus names. */
    public static final String kDefualtRioBusName = ""; 
    
    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

