package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV1;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DriverStation;

public class EndEffectorButtonCommand extends Command {
    
    private final EndEffectorV1 endEffector;
    private final double position;

    SparkAbsoluteEncoder mEncoder;

    public EndEffectorButtonCommand(double position, EndEffectorV1 endEffector)
    {
        this.position = position;
        this.endEffector = endEffector;
        this.mEncoder = endEffector.mEncoder;

        addRequirements(endEffector);

        System.out.println("Hello");
    }

    @Override
    public void initialize()
    {
        System.out.println("position "+ mEncoder.getPosition());
        endEffector.setTargetAngle(position);
        System.out.println("Urmom");
        System.out.println(endEffector.isArmAtTargetPosition()); 

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
