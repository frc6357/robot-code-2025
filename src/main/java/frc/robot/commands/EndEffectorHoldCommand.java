package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

public class EndEffectorHoldCommand extends Command{
    private final SK25EndEffector endeffector;
    double position;
    public EndEffectorHoldCommand(double position, SK25EndEffector endeffector)
    {
        this.endeffector = endeffector;
        this.position = position;
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {

        endeffector.setTargetAngle(position);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
    
}
