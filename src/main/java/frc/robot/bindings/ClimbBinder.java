package frc.robot.bindings;

import static frc.robot.Konstants.ClimbConstants.kKrakenSpeed;
import static frc.robot.Konstants.ClimbConstants.kMaxSpeed;
import static frc.robot.Konstants.ClimbConstants.kTestSpeed;
import static frc.robot.Ports.OperatorPorts.*;
import frc.robot.subsystems.SK25Climb;
import frc.robot.commands.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimbBinder implements CommandBinder {
    Optional<SK25Climb> subsystem;
    Trigger raise;
    Trigger lower;
    Trigger stop;
    Trigger slow;

    public ClimbBinder(Optional<SK25Climb> climbSys) {
        subsystem = climbSys;
        raise = climbRaiseButton.button;
        lower = climbLowerButton.button;
        stop = climbStopButton.button;
        slow = climbSlowButton.button;
    }

    public void bindButtons() 
    {
        if (subsystem.isPresent()) 
        {
            SK25Climb subsys = subsystem.get();
            raise.onTrue(new ClimbCommand1(subsys));
            lower.onTrue(new ClimbCommandReturn(subsys));
            stop.onTrue(new ClimbCommandStop(subsys));
            slow.onTrue(new ClimbCommandSlow(subsys));

            // //Press to slow
            // slow.whileTrue(new InstantCommand(() -> subsys.cambiarVelocidad(kTestSpeed)));
            // slow.onFalse(new InstantCommand(() -> subsys.cambiarVelocidad(kMaxSpeed)));

            //InstantCommand Bindings
            // raise.onTrue(new InstantCommand(() -> subsys.runMotor(kKrakenSpeed)));
            // lower.onTrue(new InstantCommand(() -> subsys.runMotor(-kKrakenSpeed)));
            // stop.onTrue(new InstantCommand(() -> subsys.runMotor(0.0)));
            // slow.onTrue(new InstantCommand(() -> subsys.runMotor(kKrakenSpeed/2)));
        }
    }
}
