package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;


import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;

import com.revrobotics.RelativeEncoder;

import static frc.robot.Konstants.EndEffectorConstants.kJoystickDeadband;

public class EndEffectorJoystickCommand extends Command {
    private final EndEffectorV2 endEffector;
    private final Supplier<Double> controller;
    RelativeEncoder mEncoder;


    public EndEffectorJoystickCommand(Supplier<Double> setpointChange, EndEffectorV2 endEffector)
    {
        this.controller = setpointChange;
        this.endEffector = endEffector;
        this.mEncoder = endEffector.mEncoder;

        addRequirements(endEffector);

        

        
    }

    @Override 
    public void initialize(){}

    @Override
    public void execute()
    {
       
       System.out.println("Joystick execute:"+controller.get()); 
       if (controller.get() > kJoystickDeadband)
       {
        double armspeed = -kArmSpeed;
        System.out.println("Position: " + mEncoder.getPosition());
         endEffector.runArm(armspeed);
         endEffector.isRunning = true;
         endEffector.checkPositionUp();
       }

       else if (controller.get() < -kJoystickDeadband)
       {
        System.out.println("Position: " + mEncoder.getPosition());
        double armspeed = kArmSpeed;
        endEffector.runArm(armspeed);
        endEffector.isRunning = true;
        endEffector.checkPositionDown();
       }

       else
       {
        endEffector.stopArm();
        endEffector.hold();
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

