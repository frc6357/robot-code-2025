package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.Angle;

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
        public static final Angle kFrontLeftEncoderOffset = Degrees.of(0.40283203125);
        public static final Angle kFrontRightEncoderOffset = Degrees.of(-0.044677734375);
        public static final Angle kBackLeftEncoderOffset = Degrees.of(-0.21875);
        public static final Angle kBackRightEncoderOffset = Degrees.of(-0.08642578125);

        //PID Constants for wheels from manual tunning
        public static final double kDriveP = 0.00001;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        //The error tolerance for the PID controllers of the wheels
        public static final double kPIDControllerTolerance = 0.1;
        
        //pigeon ID
        public static final int kPigeonID = 30; //30

        //constants which determine if the motors are inverted, negative if they are inverted, positive if not.
        public static final double kFrontRightInverted = -1.0;
        public static final double kFrontLeftInverted = -1.0;
        public static final double kBackRightInverted = -1.0;
        public static final double kBackLeftInverted = -1.0;

        //the deadzone on the controller's joysticks
        public static final double kJoystickDeadzone = 0.2;
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

