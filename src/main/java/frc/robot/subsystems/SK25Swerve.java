package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.SwerveConstants.*;


public class SK25Swerve extends SubsystemBase
{
    //declare pigeon
    public static Pigeon2 pigeon;

    //declare swerve wheels
    SwerveWheel fRWheel;
    SwerveWheel fLWheel;
    SwerveWheel bRWheel;
    SwerveWheel bLWheel;

    //declare manager
    public static SwerveManager manager;

    /**
     * Creates a new SK25Swerve Object which houses the Swerve Wheel, Swerve Manager, and the pigeon object.
     */
    public SK25Swerve()
    {
        //make new SwerveWheel objects
        fRWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD, kFrontRightDriveMotorID, kFrontRightTurnMotorID, kFrontRightEncoderID, kFrontRightEncoderOffset, kFrontRightInverted);
        fLWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD, kFrontLeftDriveMotorID, kFrontLeftTurnMotorID, kFrontLeftEncoderID, kFrontLeftEncoderOffset, kFrontLeftInverted);
        bRWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD, kBackRightDriveMotorID, kBackRightTurnMotorID, kBackRightEncoderID, kBackRightEncoderOffset, kBackRightInverted);
        bLWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD, kBackLeftDriveMotorID, kBackLeftTurnMotorID, kBackLeftEncoderID, kBackLeftEncoderOffset, kBackLeftInverted);

        //initialize pigeon
        pigeon = new Pigeon2(kPigeonID);

        //new manager object
        manager = new SwerveManager(fRWheel, fLWheel, bRWheel, bLWheel);
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
}
