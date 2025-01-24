package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbCommand1;
import frc.robot.commands.ClimbCommandReturn;
import frc.robot.subsystems.Climb;


public class ClimbBinder implements CommandBinder{
    Optional<Climb> subsystem;
    Trigger raise;
    Trigger lower;

    public ClimbBinder(Optional<Climb> climbSys) {
        subsystem = climbSys;
        raise = climbRaiseButton.button;
        lower = climbLowerButton.button;
    }

    public void bindButtons() {
        if (subsystem.isPresent()) {
            Climb subsys = subsystem.get();
            raise.onTrue(new ClimbCommand1(subsys));
            lower.onTrue(new ClimbCommandReturn(subsys));
        }
    }
}
