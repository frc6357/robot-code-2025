package frc.robot;

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

        //encoder offsets (obtained from Phoenix Tuner X)
        public static final double kFrontLeftEncoderOffset = 0.40283203125;
        public static final double kFrontRightEncoderOffset = -0.044677734375;
        public static final double kBackLeftEncoderOffset = -0.21875;
        public static final double kBackRightEncoderOffset = -0.08642578125;

        //PID Constants for wheels from manual tunning
        public static final double kDriveP = 0.00001;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        
        //pigeon ID
        public static final int kPigeonID = 30; //30
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

