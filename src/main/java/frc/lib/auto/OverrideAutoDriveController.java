package frc.lib.auto;

import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class OverrideAutoDriveController extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;

    /**
     * Creates a command that overrides the translation and rotation controllers of the
     * Pathplanner driving system. Use this command with a race condition, deadline, etc. since
     * it never ends on its own.
     * @param xSupplier The supplier for the x velocity feedback
     * @param ySupplier The supplier for the y velocity feedback
     * @param rotSupplier The supplier for the rotation velocity feedback
     */
    public OverrideAutoDriveController(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
    }

    @Override
    public void initialize() {
        PPHolonomicDriveController.overrideXYFeedback(xSupplier, ySupplier);
        PPHolonomicDriveController.overrideRotationFeedback(rotSupplier);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearXYFeedbackOverride();
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}