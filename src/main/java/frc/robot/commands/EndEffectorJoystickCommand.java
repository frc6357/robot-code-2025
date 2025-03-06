package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;
import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;

// import com.revrobotics.RelativeEncoder;

public class EndEffectorJoystickCommand extends Command {
    private final SK25EndEffector endEffector;
    private final Supplier<Double> joystickInput;


    public EndEffectorJoystickCommand(Supplier<Double> setpointChange, SK25EndEffector endEffector)
    {
        this.joystickInput = setpointChange;
        this.endEffector = endEffector;

        addRequirements(endEffector);

        

        
    }

    @Override 
    public void initialize(){}

    @Override
    public void execute()
    {
         
        
       if (joystickInput.get() > 0)
       {
        double armspeed = kArmSpeed;
        double armdividend = joystickInput.get();
        armspeed = armspeed * armdividend;
         endEffector.runArm(armspeed);
         endEffector.isRunning = true;
         endEffector.checkPositionUp();
         
       }

       else if (joystickInput.get() < 0)
       {
        double armdividend = joystickInput.get();
        double armspeed = -kArmSpeed;
        armspeed = armspeed * armdividend;
        endEffector.runArm(-armspeed);
        endEffector.isRunning = true;
        endEffector.checkPositionDown();
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

