package frc.robot.subsystems.drive;

import java.util.function.BiFunction;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.lib.utils.DriveJoystickInput;
import frc.robot.Konstants.DriveConstants;

/**
 * Contains various swerve drive requests to be used by the drive subsystem.
 */
public class DriveRequests {
    public static final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final BiFunction<SwerveRequest.FieldCentric, DriveJoystickInput, SwerveRequest.FieldCentric> teleopRequestUpdater =
        (SwerveRequest.FieldCentric request, DriveJoystickInput joystickInputs) -> {
            double xDesired = joystickInputs.getXJoystick();
            double yDesired = joystickInputs.getYJoystick();
            double rotDesired = joystickInputs.getRotationJoystick();

            if(joystickInputs.isSlow()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedSLOW.times(xDesired))
                        .withVelocityY(DriveConstants.kMaxSpeedSLOW.times(yDesired))
                        .withRotationalRate(DriveConstants.kMaxAngularRateSLOW.times(rotDesired));
            }
            if(joystickInputs.isFast()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedFAST.times(xDesired))
                        .withVelocityY(DriveConstants.kMaxSpeedFAST.times(yDesired))
                        .withRotationalRate(DriveConstants.kMaxAngularRateFAST.times(rotDesired));
            }

            return request
                    .withVelocityX(DriveConstants.kMaxSpeed.times(xDesired))
                    .withVelocityY((DriveConstants.kMaxSpeed).times(yDesired))
                    .withRotationalRate(DriveConstants.kMaxAngularRate.times(rotDesired));
        };

}
