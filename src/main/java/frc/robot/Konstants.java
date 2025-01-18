package frc.robot;

public final class Konstants
{

    public static final class SwerveConstants
    {
        //8 swerve motor IDs for Ports
        public static final int kFrontLeftDriveMotorID = 1; 
        public static final int kFrontRightDriveMotorID = 2;
        public static final int kBackLeftDriveMotorID = 3;
        public static final int kBackRightDriveMotorID = 4;

        public static final int kFrontLeftTurnMotorID = 11;
        public static final int kFrontRightTurnMotorID = 12;
        public static final int kBackLeftTurnMotorID = 13;
        public static final int kBackRightTurnMotorID = 14;

        public static final int kFrontLeftEncoderID = 21;
        public static final int kFrontRightEncoderID = 22;
        public static final int kBackLeftEncoderID = 23;
        public static final int kBackRightEncoderID = 24;

        //swerve chassis width and length in inches
        public static final int kChassisLength = 28;
        public static final int kChassisWidth = 28;

        //PID Constants for wheels
        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        
        //pigeon ID
        public static final int kPigeonID = 30;
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

