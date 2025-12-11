package frc.lib.auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoRotationControllerOverride extends Command {
    private final DoubleSupplier headingSupplier;

    public AutoRotationControllerOverride(DoubleSupplier headingSupplier) {
        this.headingSupplier = headingSupplier;
    }

    @Override
    public void initialize() {
        PPHolonomicDriveController.overrideRotationFeedback(headingSupplier);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}
