package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV1;

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
        double angleChange = controller.get() / 50;

        double armSetpoint = endEffector.getTargetArmPosition() + angleChange;

        endEffector.setTargetAngle(armSetpoint);

        System.out.println("exec");
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
