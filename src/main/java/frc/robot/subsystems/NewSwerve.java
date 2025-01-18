package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.SwerveConstants.*;


public class NewSwerve extends SubsystemBase
{
    //declare pigeon
    public static Pigeon2 pigeon;

    //swerve wheels
    SwerveWheel fRWheel;
    SwerveWheel fLWheel;
    SwerveWheel bRWheel;
    SwerveWheel bLWheel;

    //manager
    public static SwerveManager manager;

    public NewSwerve()
    {
        fRWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD);
        fLWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD);
        bRWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD);
        bLWheel = new SwerveWheel(kDriveP, kDriveI, kDriveD);

        //initialize pigeon
        pigeon = new Pigeon2(kPigeonID);

        //manager
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
