package frc.robot.commands;

import static frc.robot.Konstants.ClimbConstants.*;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25Climb;

public class ClimbCommand1 extends Command{
   public final SK25Climb climb; 
    
    public ClimbCommand1(SK25Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climb.runMotor(kVolts);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }


}