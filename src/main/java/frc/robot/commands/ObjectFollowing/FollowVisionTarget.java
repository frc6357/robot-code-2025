package frc.robot.commands.ObjectFollowing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.vision.Limelight;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.SKSwerve;
import frc.robot.commands.DriveCommand;

public class FollowVisionTarget extends Command{
    public static enum VisionTarget {
        APRIL_TAG,
        NEURAL_TARGET
    }
    VisionTarget target;
    Limelight[] limelights;
    Limelight limelight;
    SK25Vision m_vision;
    SKSwerve m_swerve;
    PIDController rotPID;
    DriveCommand driveCommand;
    double rotOut;

    public FollowVisionTarget(VisionTarget target, SKSwerve m_swerve, SK25Vision m_vision) {
        this.target = target;
        this.m_swerve = m_swerve;
        this.m_vision = m_vision;
        rotPID = new PIDController(0.06, 0, 0.0015);

        // Should prob make this a switch statement later to avoid excess "else if's"
        if(target == VisionTarget.APRIL_TAG) {
            limelights = m_vision.poseLimelights;
        }
        else if(target == VisionTarget.NEURAL_TARGET) {
            limelights = m_vision.detectLimelights;
        }

        this.driveCommand = new DriveCommand(
            () -> 0.0, 
            () -> 0.0, 
            () -> getOutput(), 
            () -> false);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        limelight = findGoodLimelight();

        // All rotation values assumed to be degrees
        rotPID.setSetpoint(0);
        rotPID.setTolerance(0.1);
        rotPID.reset();
    }

    @Override
    public void execute() {
        if(limelight == null) {
            return;
        }

        m_vision.isDriving = true;

        rotOut = rotPID.calculate(limelight.getHorizontalOffset());
        driveCommand.run();
    }

    @Override
    public boolean isFinished() {
        if(limelight != null) {
            return !limelight.targetInView();
        }
        return (limelight == null);
    }

    @Override
    public void end(boolean isInterrupted) {
        m_vision.isDriving = false;
    }

    private double getOutput() {
        return rotOut;
    }

    // Finds any limelight with a valid target in view
    private Limelight findGoodLimelight() {
        for(Limelight ll : limelights) {
            if(ll.targetInView()) {
                return ll;
            }
        }

        // Careful with this null return
        return null;
    }
}
