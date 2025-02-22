package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;

public class EndEffectorButtonCommand extends Command{

    private final EndEffectorV2 endEffector;
    private final double position;

    RelativeEncoder mEncoder;

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
        System.out.println("Position: " + position);
        System.out.println("Encoder position: " + mEncoder.getPosition());
        endEffector.setTargetAngle(-position);
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
            System.out.println(endEffector.isArmAtTargetPosition());
            //System.out.println(Math.abs( endEffector.getTargetArmPosition() - endEffector.getArmPosition()));
            //System.out.println(endEffector.getTargetArmPosition());
            //System.out.println(endEffector.getArmPosition());

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