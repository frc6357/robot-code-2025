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
        double joystickFilteredInput = joystickInput.get();

        if(Math.abs(joystickFilteredInput) > 0) { // If it's greater than 0, it's already overcome the deadband set in the command Binder
            double armspeed = Math.signum(joystickFilteredInput) * kArmSpeed;
            double armdividend = joystickFilteredInput;
            armspeed *= (armdividend/3);

            endEffector.runArm(armspeed);
            endEffector.isRunning = true;
        }
        else {
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

