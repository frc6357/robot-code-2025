package frc.lib.auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoTranslationControllerOverride extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public AutoTranslationControllerOverride(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    @Override
    public void initialize() {
        PPHolonomicDriveController.overrideXYFeedback(xSupplier, ySupplier);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearXYFeedbackOverride();
    }
}
