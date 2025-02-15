package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;


import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;

public class EndEffectorButtonCommand extends Command{

    private final EndEffectorV2 endEffector;
    private final double position;

    SparkRelativeEncoder mEncoder;

    public EndEffectorButtonCommand(double position, EndEffectorV2 endEffector)
    {
        this.position = position;
        this.endEffector = endEffector;
        this.mEncoder = endEffector.mEncoder;

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
            if(endEffector.isArmAtTargetPosition())
            {
                
                return true;
            }
            else
            {
                return false;
            }
        }

    }
}
    
