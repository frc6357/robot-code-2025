package frc.robot;

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

    public static final class ElevatorConstants
    {
        /** Heights for the different elevator positions */
        public static enum ElevatorPosition
        {
            //TODO FIX_DURING_TESTING - Measure Elevator Heights
            /** Set the height to reach the top branch (L4) */
            TopPosition(27),
            /** Set the height to reach the middle branch (L3) */
            MidPosition(22),
            /** Set the height to reach the low branch (L2) */
            LowPosition(17),
            /** Set the height to reach the trough (L1) */
            TroughPosition(12),
            /** Set the height to reach the bottom */
            ZeroPosition(0.0);

            public final double height;

            ElevatorPosition(double height)
            {
                this.height = height;
            }
        }

        public static final PIDConstants rightElevator = new PIDConstants(0.07, 0.00075, 0.001);
        public static final PIDConstants leftElevator = new PIDConstants(0.07, 0.00075, 0.001);
        public static final PIDConstants balancePID = new PIDConstants(0.0, 0.0, 0.0);

        public static final double kMinInteg = 0.0;
        public static final double kMaxInteg = 0.15;

        public static final double kPositiveAccelLimit = 1.0; // in %/sec
        public static final double kNegativeAccelLimit = -5.0; // in %/sec

        public static final double kElevatorBalanceTolerance = 5.0;
        public static final double spoolDiameter = 1.273; //Inches
        public static final double gearRatio = .1264; //Shaft rotations / 1 motor rotation
        public static final double elevatorHeight = 28; //Inches

        public static final double elevatorConversion = 1.0 / 87.0; //inches moved per motor rotation
        public static final double kPositionTolerance = 2.0;
        public static final double kElevatorMotorMinOutput = -0.5;
        public static final double kElevatorMotorMaxOutput = 0.8;

        public static final double kElevatorUpSpeed = 1.0;
        public static final double kElevatorDownSpeed = -1.0;

        public static final double kCANCoderGearRatio = 160.0 / 48.0; //Convert encoder degree units to arm degrees

        public static final int kElevatorCurrentLimit = 30;

        //public static final double kElevatorUpSpeedLeft = -1.0;
        //public static final double kElevatorDownSpeedLeft = 1.0;

        public static final double kMaxHeight = 28;
        public static final double kMinHeight = 0;

        public static final double kJoystickChange   = 2.0; // Manual setpoint value for units from 0.0 - 1.0 moved per second
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

