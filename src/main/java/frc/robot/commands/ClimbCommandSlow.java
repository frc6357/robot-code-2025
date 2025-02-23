package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Konstants.ClimbConstants.*;
import frc.robot.subsystems.SK25Climb;

public class ClimbCommandSlow extends Command{
   public final SK25Climb climb; 
    
    public ClimbCommandSlow(SK25Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
       // climb.cambiarVelocidad(kTestSpeed);
    }

    @Override
    public void execute() {
        climb.cambiarVelocidad(kTestSpeed);
    }

    @Override
    public void end(boolean interrupted) {
       // climb.cambiarVelocidad(kMaxSpeed);
    }

    @Override
    public boolean isFinished() {
       return true;
    }


}