package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV1;


import edu.wpi.first.wpilibj.DriverStation;

public class EndEffectorButtonCommand extends Command {
    
    private final EndEffectorV1 endEffector;
    private final double position;

    public EndEffectorButtonCommand(double position, EndEffectorV1 endEffector)
    {
        this.position = position;
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

    @Override
    public void initialize()
    {
        endEffector.setTargetAngle(position);
    }

    @Override
    public boolean isFinished()
    {
        if(DriverStation.isAutonomousEnabled())
        {
            if(endEffector.isArmAtTargetPosition())
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return true;
        }
    }

}
