package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.MecanumDriveConstants.kFrontLeftLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kFrontRightLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kBackLeftLocation;
import static frc.robot.Konstants.MecanumDriveConstants.kBackRightLocation;
import static frc.robot.Ports.DrivePorts.kPigeonPort;

import com.ctre.phoenix6.hardware.Pigeon2;


public class SKMecanumDrive extends SubsystemBase {
    Pigeon2 m_pigeon;
    // Creating kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics;
    MecanumDriveOdometry m_odometry;


    public SKMecanumDrive() {
        m_pigeon = new Pigeon2(kPigeonPort.ID, kPigeonPort.bus);

        m_kinematics = new MecanumDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation,
            kBackLeftLocation, kBackRightLocation
        );

        m_odometry = new MecanumDriveOdometry(
            m_kinematics,
            getIMURotation(),
            getWheelPositions(),
            new Pose2d(0.0, 0.0, new Rotation2d())
        );
    }

    @Override
    public void periodic() {
        m_odometry.update(getIMURotation(), getWheelPositions());
    }

    /**
     * Sets the <b>robot-relative</b> velocity for the drivetrain to target.
     * @param vx Translational X velocity
     * @param vy Translation Y velocity
     * @param omega Rotational velocity (rad/sec)
     * @return
     */
    public void setControl(double vx, double vy, double omega) {

    }

    /**
     * Sets the <b>robot-relative</b> velocity for the drivetrain to target.
     * @param speeds The ChassisSpeeds object to handle
     */
    public void setControl(ChassisSpeeds speeds) {
        
    }

    // Uses the pigeon IMU to return the robot's rotation
    public Rotation2d getIMURotation() {
        return m_pigeon.getRotation2d();
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
            m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
            m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
        );
    }

    public void resetIMU() {
        m_pigeon.reset();
    }

    /**
     * Resets both the IMU and odometry rotation
     */
    public void resetRotation() {
        m_pigeon.reset();
        m_odometry.resetRotation(getIMURotation());
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPose(pose);
    }
}
