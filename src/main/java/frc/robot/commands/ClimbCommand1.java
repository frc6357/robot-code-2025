package frc.robot.commands;

import static frc.robot.Konstants.ClimbConstants.kClimbSetpoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommand1 extends Command{
   public final Climb climb; 
    
    public ClimbCommand1(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climb.setPointL(kClimbSetpoint);
        climb.setPointR(kClimbSetpoint);
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