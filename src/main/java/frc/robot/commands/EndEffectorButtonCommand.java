package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition;

public class EndEffectorButtonCommand extends Command{

    private final SK25EndEffector endEffector;
    private final EndEffectorPosition angle;


    public EndEffectorButtonCommand(EndEffectorPosition angle, SK25EndEffector endEffector)
    {
        this.angle = angle;
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

    @Override
    public void initialize()
    {
        //System.out.println("Position: " + position);
       // System.out.println("Encoder position: " + mEncoder.getPosition());
        endEffector.setTargetAngle(angle.angle);
        endEffector.isRunning = true;
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
            // System.out.println(endEffector.isArmAtTargetPosition());
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