package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.SK25Climb;




public class ClimbBinder implements CommandBinder {
    Optional<SK25Climb> subsystem;
    Trigger raise;
    Trigger lower;
    Trigger stop;

    public ClimbBinder(Optional<SK25Climb> climbSys) {
        subsystem = climbSys;
        raise = climbRaiseButton.button;
        lower = climbLowerButton.button;
        stop = climbStopButton.button;
    }

    public void bindButtons() 
    {
        if (subsystem.isPresent()) 
        {
            SK25Climb subsys = subsystem.get();
         raise.onTrue(new ClimbCommand1(subsys));
         lower.onTrue(new ClimbCommandReturn(subsys));
         stop.onTrue(new ClimbCommandStop(subsys));

         //  raise.onTrue(new InstantCommand(() -> subsys.runMotor(kSpeed)));
         //  lower.onTrue(new InstantCommand(() -> subsys.runMotor(-kSpeed)));
        }
    }
}
