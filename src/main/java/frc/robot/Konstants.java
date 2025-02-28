package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

@SuppressWarnings("unused")
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

    public static final class ClimbConstants
    {
        //Keeping P value at 0 will result in motor not spinning
        public static final double kClimbP = 1.0;         //TODO: tune climb PID
        public static final double kClimbI = 0.0;
        public static final double kClimbD = 0.0;
      //  public static final double kClimbSetpoint = 5.0;
        public static final double kClimbStartPos = 0.0;
        public static final double kKrakenSpeed = .1;
        public static final double kMaxSpeed = 1000;
        public static final double kTestSpeed = 300;
        public static final double kVolts = 3;
      //  public static final double kMaxSpeed = 1000;
      //  public static final double kTestSpeed = 300;
        public static final double kClimbMaxAcceleration = 1000;
        public static final int kClimbCurrentLimit = 50;
        public static final double kClimbMaxPosition = 30.0;
     //   public static final Rotation2d sigma = Rotation2d.fromDegrees(3600);
       // public static final Angle kAngleMax = kClimbMaxPosition.in(Units.Angle);
        public static final double kClimbMinPosition = 0.0;
        public static final double kClimbPositionTolerance = 0.1;

    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;
    }
    
    
 
    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

