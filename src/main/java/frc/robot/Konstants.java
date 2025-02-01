package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;



public final class Konstants
{

    public static final class PracticeSwerveConstants
    {
        //8 swerve motor IDs for Ports
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftTurnMotorId = 2;
        public static final int kFrontRightDriveMotorId = 3;
        public static final int kFrontRightTurnMotorId = 4;
        public static final int kBackLeftDriveMotorId = 5;
        public static final int kBackLeftTurnMotorId = 6;
        public static final int kBackRightDriveMotorId = 7;
        public static final int kBackRightTurnMotorId = 8;

        //TODO: Update IDs for 2025
        public static final int kFrontLeftEncoderId = 33;
        public static final int kFrontRightEncoderId = 31;
        public static final int kBackLeftEncoderId = 32;
        public static final int kBackRightEncoderId = 30;

        //swerve chassis width and length in inches
        public static final int kChassisLength = 28;
        public static final int kChassisWidth = 28;

        //PID Constants
        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0;
        public static final double kDriveD = 0;
        public static final double kTurnP = 0.1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        //TODO: THIS IS 2024 TUNING!!!! PLEASE UPDATE ME!!!!
        public static final Slot0Configs steerGains = new Slot0Configs() //TODO - tune steering gains drive
        .withKP(100).withKI(0).withKD(0.3)
        .withKS(0).withKV(1.5).withKA(0);

        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        //TODO: THIS IS 2024 TUNING!!!! PLEASE UPDATE ME!!!!
        public static final Slot0Configs driveGains = new Slot0Configs() //TODO - tune driving gains drive
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
        public static final double kWheelRadiusInches = 2.0 - 0.1;

        public static final boolean kInvertLeftSide = false;
        public static final boolean kSteerMotorReversed = true;
        public static final boolean kInvertRightSide = true;

        //TODO: Check these values
        public static final String kCANbusName = "DriveCAN";
        public static final int kPigeonId = 25;

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        public static final double kWheelRadius = (3.815 / 2); //TODO: WHEEL RADIUS FROM SPECTRUM, UPDATE WITH OUR OWN
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

    public static final class LightConstants
    {
        public static final int numLedOnBot = 240;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;
    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;
    }
    
    

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

