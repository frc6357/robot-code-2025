package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorV2;
import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;

//import static frc.robot.Konstants.EndEffectorConstants.kArmTolerance;

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
       
        
       if (controller.get() > kJoystickDeadband)
       {
        double armspeed = -kArmSpeed;
        double armdividend = controller.get();
        armspeed /= 3;
        armspeed = armspeed * armdividend;
        System.out.println("Position: " + mEncoder.getPosition());
         endEffector.runArm(armspeed);
         endEffector.isRunning = true;
         //endEffector.checkPositionUp();
         System.out.println("Joystick execute:"+controller.get());
       }

       else if (controller.get() < -kJoystickDeadband)
       {
        double armdividend = controller.get();
        System.out.println("Position: " + mEncoder.getPosition());
        double armspeed = kArmSpeed / 3;
        armspeed = armspeed * armdividend;
        endEffector.runArm(-armspeed);
        endEffector.isRunning = true;
        System.out.println("Joystick execute:"+controller.get());
        //endEffector.checkPositionDown();
       }

       else
       {
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

