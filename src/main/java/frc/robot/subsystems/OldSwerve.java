// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import static frc.robot.Konstants.SwerveConstants.*;


// public class OldSwerve extends SubsystemBase
// {
//     //declare pigeon
//     public static Pigeon2 pigeon;

//     //declare swerve wheels
//     OldSwerveWheel fRWheel;
//     OldSwerveWheel fLWheel;
//     OldSwerveWheel bRWheel;
//     OldSwerveWheel bLWheel;

//     //declare manager
//     public static OldSwerveManager manager;

//     /**
//      * Creates a new SK25Swerve Object which houses the Swerve Wheel, Swerve Manager, and the pigeon object.
//      */
//     public OldSwerve()
//     {
//         //make new SwerveWheel objects
//         fRWheel = new OldSwerveWheel(kDriveP, kDriveI, kDriveD, kFrontRightDriveMotorID, kFrontRightTurnMotorID, kFrontRightEncoderID, kFrontRightEncoderOffset, kFrontRightInverted);
//         fLWheel = new OldSwerveWheel(kDriveP, kDriveI, kDriveD, kFrontLeftDriveMotorID, kFrontLeftTurnMotorID, kFrontLeftEncoderID, kFrontLeftEncoderOffset, kFrontLeftInverted);
//         bRWheel = new OldSwerveWheel(kDriveP, kDriveI, kDriveD, kBackRightDriveMotorID, kBackRightTurnMotorID, kBackRightEncoderID, kBackRightEncoderOffset, kBackRightInverted);
//         bLWheel = new OldSwerveWheel(kDriveP, kDriveI, kDriveD, kBackLeftDriveMotorID, kBackLeftTurnMotorID, kBackLeftEncoderID, kBackLeftEncoderOffset, kBackLeftInverted);

//         //initialize pigeon
//         pigeon = new Pigeon2(kPigeonID);

//         //new manager object
//         manager = new OldSwerveManager(fRWheel, fLWheel, bRWheel, bLWheel);
//     }

//     //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
//     public void periodic()
//     {
//     } 

//     public void testInit()
//     {
//     }
    
//     public void testPeriodic()
//     {
//     }
// }
