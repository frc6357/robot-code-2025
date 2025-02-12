package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV1;

import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;
import static frc.robot.Ports.OperatorPorts.endArm;

public class EndEffectorJoystickCommand extends Command{

    private final EndEffectorV1 endEffector;
    private final Supplier<Double> controller;


    public EndEffectorJoystickCommand(Supplier<Double> setpointChange, EndEffectorV1 endEffector)
    {
        this.controller = setpointChange;
        this.endEffector = endEffector;

        addRequirements(endEffector);

        System.out.println("Hii");
    }

    @Override 
    public void initialize(){}

    @Override
    public void execute()
    {
        
       if (controller.get() > 0)
       {
         endEffector.runArm(kArmSpeed);
       }

       if (controller.get() < 0)
       {
        double armspeed = -kArmSpeed;
        endEffector.runArm(armspeed);
       }

       if(controller.get() == 0)
       {
        endEffector.stopArm();
       }

    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
