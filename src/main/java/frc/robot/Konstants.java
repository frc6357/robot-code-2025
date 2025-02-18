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
            //TODO FIX DURING TESTING - Measure Elevator Heights

            /** Set the height to reach the top branch (L4) */
            kTopPosition(48),
            /** Set the height to reach the middle branch (L3) */
            kMidPosition(36),
            /** Set the height to reach the low branch (L2) */
            kLowPosition(8),
            /** Set the height to reach the trough (L1) */
            kTroughPosition(4),
            /** Set the height to reach the bottom */
            kZeroPosition(0.0);

            public final double height;

            ElevatorPosition(double height)
            {
                this.height = height;
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
        public static final double kPositionTolerance = 2.0;

        // Minimum & Maximum Outputs For Elevator
        public static final double kElevatorMotorMinOutput = -0.5;
        public static final double kElevatorMotorMaxOutput = 0.8;

        // Maximum Current Limit For The ELevator
        public static final int kElevatorCurrentLimit = 30;
        
        /*
        Minumum & Maximum Heights The Elevator Can Be Within
        TODO Change the height and see how that works, check SmartDashboard for elevator values first.
        */
        public static final double kMaxHeight = 70;
        public static final double kMinHeight = 0;

        // Important Joystick Settings
        public static final double kJoystickChange   = 10.0;
        public static final double kJoystickDeadband = 0.2;  // Manual elevator movement axis deadband
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

