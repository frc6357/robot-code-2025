package frc.robot.subsystems;

import static frc.robot.Konstants.SwerveConstants.kBackLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kBackLeftInverted;
import static frc.robot.Konstants.SwerveConstants.kBackRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kBackRightInverted;
import static frc.robot.Konstants.SwerveConstants.kCANivoreName;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kFrontLeftInverted;
import static frc.robot.Konstants.SwerveConstants.kFrontRightEncoderOffsetRadians;
import static frc.robot.Konstants.SwerveConstants.kFrontRightInverted;
import static frc.robot.Ports.DrivePorts.kBackLeftDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kBackLeftEncoderPort;
import static frc.robot.Ports.DrivePorts.kBackLeftTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kBackRightDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kBackRightEncoderPort;
import static frc.robot.Ports.DrivePorts.kBackRightTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftEncoderPort;
import static frc.robot.Ports.DrivePorts.kFrontLeftTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontRightDriveMotorPort;
import static frc.robot.Ports.DrivePorts.kFrontRightEncoderPort;
import static frc.robot.Ports.DrivePorts.kFrontRightTurnMotorPort;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK25Swerve extends SubsystemBase{

    public static SK25SwerveFactory factory;

    public static Pigeon2 pigeon;

    SK25SwerveModule fLModule;
    SK25SwerveModule fRModule;
    SK25SwerveModule bLModule;
    SK25SwerveModule bRModule;

    public SK25Swerve()
    {
        fLModule = new SK25SwerveModule(kFrontLeftDriveMotorPort.ID, kFrontLeftTurnMotorPort.ID, kFrontLeftEncoderPort.ID, kFrontLeftEncoderOffsetRadians, kFrontLeftInverted);
        fRModule = new SK25SwerveModule(kFrontRightDriveMotorPort.ID, kFrontRightTurnMotorPort.ID, kFrontRightEncoderPort.ID, kFrontRightEncoderOffsetRadians, kFrontRightInverted);
        bLModule = new SK25SwerveModule(kBackLeftDriveMotorPort.ID, kBackLeftTurnMotorPort.ID, kBackLeftEncoderPort.ID, kBackLeftEncoderOffsetRadians, kBackLeftInverted);
        bRModule = new SK25SwerveModule(kBackRightDriveMotorPort.ID, kBackRightTurnMotorPort.ID, kBackRightEncoderPort.ID, kBackRightEncoderOffsetRadians, kBackRightInverted);

        pigeon = new Pigeon2(kPigeonPort.ID, kCANivoreName);

        factory = new SK25SwerveFactory(fLModule, fRModule, bLModule, bRModule, pigeon);
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
        factory.periodic();
    } 

    public void testInit()
    {
        factory.testInit();
    }
    
    public void testPeriodic()
    {
        factory.testPeriodic();
    }
}
