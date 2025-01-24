package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommandReturn extends Command{
   public final Climb climb; 
    
    public ClimbCommandReturn(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climb.setPointL(0.0);
        climb.setPointR(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        climb.runLeftHook(0);
        climb.runRightHook(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}