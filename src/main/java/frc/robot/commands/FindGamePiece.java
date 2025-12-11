package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;

public class FindGamePiece extends Command{
    double xOutput;
    double yOutput;
    double rotOutput;
    double durationSeconds;
    SKSwerve swerve;
    Timer timer = new Timer();

    public FindGamePiece(
        double durationSeconds,
        SKSwerve swerve,
        SK25Vision vision
        ) 
    {
        this.durationSeconds = durationSeconds;
        this.swerve = swerve;

        timer.reset();
        addRequirements(swerve);
    }

    // Calculate the speeds needed to approach the game piece via vision
    private void calculateSpeeds() {
        // Placeholder values for now
        xOutput = 1.0;
        yOutput = 1.0;
        rotOutput = 1 * Math.PI;
    }

    @Override
    public void initialize() {
        PPHolonomicDriveController.overrideXYFeedback(() -> xOutput, () -> yOutput);
        PPHolonomicDriveController.overrideRotationFeedback(() -> rotOutput);

        timer.start();
    }

    @Override
    public void execute() {
        calculateSpeeds();
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}
