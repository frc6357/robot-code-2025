package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.*;
import frc.robot.subsystems.SK25Climb;
import frc.robot.commands.*;

import java.util.Optional;

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
            raise.whileTrue(new ClimbCommand1(subsys));
            lower.whileTrue(new ClimbCommandReturn(subsys));
           // stop.whileTrue(new ClimbCommandStop(subsys));
            // slow.whileTrue(new ClimbCommandSlow(subsys));
            // slow.onFalse(new InstantCommand(() -> subsys.runMotor(kVolts)));

             //No-Eyeballing
               //raise.onTrue(new InstantCommand(() -> subsys.setSetpoint(sigma.getMeasure())));
            //Angle.in(Degrees.of(620));
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
