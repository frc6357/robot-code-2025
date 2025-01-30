package frc.robot.commands;

import static frc.robot.Konstants.ClimbConstants.kSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Climb;

public class ClimbCommandReturn extends Command{
   public final SK25Climb climb; 
    
    public ClimbCommandReturn(SK25Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climb.setPoint(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        climb.runMotor(kSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}