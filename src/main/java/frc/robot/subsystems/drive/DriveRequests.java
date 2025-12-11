package frc.robot.subsystems.drive;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Konstants.DriveConstants;

/**
 * Contains various swerve drive requests to be used by the drive subsystem.
 */
public class DriveRequests {
    public static final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriveConstants.kMaxSpeed.times(0.1));

    public static final UnaryOperator<SwerveRequest.FieldCentric> getTeleopRequestUpdater(
        Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rotJoystick, Supplier<Boolean> slow, Supplier<Boolean> fast
    ) {
        return (SwerveRequest.FieldCentric request) -> {
            SmartDashboard.putNumberArray("Drive/Joystick Inputs", new double[] {xJoystick.get(), yJoystick.get(), rotJoystick.get()});
            if(slow.get()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedSLOW.times(xJoystick.get()))
                        .withVelocityY(DriveConstants.kMaxSpeedSLOW.times(yJoystick.get()))
                        .withRotationalRate(DriveConstants.kMaxAngularRateSLOW.times(rotJoystick.get()));
            }
            if(fast.get()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedFAST.times(xJoystick.get()))
                        .withVelocityY(DriveConstants.kMaxSpeedFAST.times(yJoystick.get()))
                        .withRotationalRate(DriveConstants.kMaxAngularRateFAST.times(rotJoystick.get()));
            }
            return request
                    .withVelocityX(DriveConstants.kMaxSpeed.times(xJoystick.get()))
                    .withVelocityY((DriveConstants.kMaxSpeed).times(yJoystick.get()))
                    .withRotationalRate(DriveConstants.kMaxAngularRate.times(rotJoystick.get()));
        };
    }

    public static final SwerveRequest.RobotCentric robotCentricTeleopRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriveConstants.kMaxSpeed.times(0.1));
    
    public static final UnaryOperator<SwerveRequest.RobotCentric> getRobotCentricTeleopRequestUpdater(
        Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rotJoystick, Supplier<Boolean> slow, Supplier<Boolean> fast
    ) {
        return (SwerveRequest.RobotCentric request) -> {
            SmartDashboard.putNumberArray("Drive/Joystick Inputs", new double[] {xJoystick.get(), yJoystick.get(), rotJoystick.get()});
            if(slow.get()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedSLOW.times(xJoystick.get()))
                        .withVelocityY(DriveConstants.kMaxSpeedSLOW.times(yJoystick.get()))
                        .withRotationalRate(DriveConstants.kMaxAngularRateSLOW.times(rotJoystick.get()));
            }
            if(fast.get()) {
                return request
                        .withVelocityX(DriveConstants.kMaxSpeedFAST.times(xJoystick.get()))
                        .withVelocityY(DriveConstants.kMaxSpeedFAST.times(yJoystick.get()))
                        .withRotationalRate(DriveConstants.kMaxAngularRateFAST.times(rotJoystick.get()));
            }
            return request
                    .withVelocityX(DriveConstants.kMaxSpeed.times(xJoystick.get()))
                    .withVelocityY((DriveConstants.kMaxSpeed).times(yJoystick.get()))
                    .withRotationalRate(DriveConstants.kMaxAngularRate.times(rotJoystick.get()));
        };
    }
    
    public static final SwerveRequest.ApplyRobotSpeeds pathPlannerRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    public static final UnaryOperator<SwerveRequest.ApplyRobotSpeeds> getPathPlannerRequestUpdater(
        Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) { 
        return (SwerveRequest.ApplyRobotSpeeds request) -> {
            return request
                .withSpeeds(speeds.get())
                .withWheelForceFeedforwardsX(feedforwards.get().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.get().robotRelativeForcesYNewtons());
        };
    }

    // public static final UnaryOperator<SwerveRequest.RobotCentric> getPathPlannerRequestUpdater(
    //     Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) { 
    //     return (SwerveRequest.RobotCentric request) -> {
    //         return request
    //             .withVelocityX(speeds.get().vxMetersPerSecond)
    //             .withVelocityY(speeds.get().vyMetersPerSecond)
    //             .withRotationalRate(speeds.get().omegaRadiansPerSecond);
    //     };
    // }
}
