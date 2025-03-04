package frc.robot.commands;

import static frc.robot.Konstants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Climb;

public class ClimbCommandStop extends Command{
   public final SK25Climb climb; 
    
    public ClimbCommandStop(SK25Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.stop();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}