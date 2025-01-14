package frc.robot;


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

