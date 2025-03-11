package frc.robot.bindings;

// import frc.robot.RobotContainer.m_vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToReef.DriveToReefCommand;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;

import static frc.robot.Ports.DriverPorts.kDriveToClosestReef;
import static frc.robot.Ports.DriverPorts.kLeftReef;
import static frc.robot.Ports.DriverPorts.kRightReef;

public class SK25VisionBinder implements CommandBinder {
    Optional<SK25Vision> m_visionContainer;
    Optional<SKSwerve> m_swerveContainer;

    Trigger driveToClosestReef;
    Trigger driveToLeftReef;
    Trigger driveToRightReef;

    public SK25VisionBinder(Optional<SK25Vision> m_visionContainer, Optional<SKSwerve> m_swerveContainer) {
        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;

        this.driveToClosestReef = kDriveToClosestReef.button;
        this.driveToLeftReef = kLeftReef.button;
        this.driveToRightReef = kRightReef.button;
    }

    public void bindButtons() {
        if(m_visionContainer.isPresent() && m_swerveContainer.isPresent()) {
            // The specific swerve instance is needed in order to control the robot
            // while the vision commands are all static since vision doesn't need one specific
            // instance to be controlled. Vision should be able to run multiple commands
            // either in sequence or parallel with itself.
            SKSwerve m_swerve = m_swerveContainer.get();
            SK25Vision m_vision = m_visionContainer.get();

            /* This command feeds triggers into its constructor in order for it  */
            driveToClosestReef.whileTrue(
                new DriveToReefCommand(
                    SK25Vision.DriveToPose.getConfig(),
                    SK25Vision.RotateToPose.getConfig(), 
                    m_vision, 
                    m_swerve,
                    driveToLeftReef,
                    driveToRightReef));
        }
    }
}
