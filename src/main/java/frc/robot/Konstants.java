package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Konstants
{

    // public static final class PracticeSwerveConstants
    // {
    //     //8 swerve motor IDs for Ports
    //     public static final int kFrontLeftDriveMotorId = 1;
    //     public static final int kFrontLeftTurnMotorId = 2;
    //     public static final int kFrontRightDriveMotorId = 3;
    //     public static final int kFrontRightTurnMotorId = 4;
    //     public static final int kBackLeftDriveMotorId = 5;
    //     public static final int kBackLeftTurnMotorId = 6;
    //     public static final int kBackRightDriveMotorId = 7;
    //     public static final int kBackRightTurnMotorId = 8;

    //     //swerve chassis width and length in inches
    //     public static final int kChassisLength = 28;
    //     public static final int kChassisWidth = 28;

    //     //PID Constants
    //     public static final double kDriveP = 0.1;
    //     public static final double kDriveI = 0;
    //     public static final double kDriveD = 0;
    //     public static final double kTurnP = 0.1;
    //     public static final double kTurnI = 0;
    //     public static final double kTurnD = 0;
    // }

    public static final class MecanumDriveConstants {
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.5);

        public static final double kMaxSpeed = kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        public static final double kMaxAngularSpeedDegrees = RadiansPerSecond.of(kMaxAngularSpeed).in(DegreesPerSecond);

        public static final int kPigeonID = kPigeonPort.ID;

        // Locations of the wheels relative to the robot center. (In meters)
        public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
        public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);
        public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
    }

    public static final class IOConstants {
        public static final double kJoystickDeadband = 0.15;
        public static final double kSlowModePercent  = 0.3;
        public static final double kSlowModeRotationPercent = 0.5;
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

