package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.climbLowerButton;
import static frc.robot.Ports.DriverPorts.climbRaiseButton;
import static frc.robot.Ports.DriverPorts.climbSlowButton;
import static frc.robot.Ports.DriverPorts.climbStopButton;
import static frc.robot.Konstants.ClimbConstants.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25Climb;

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

            //Run climb
            raise.whileTrue(new InstantCommand(() -> subsys.runMotor(-kKrakenSpeed)));
            lower.whileTrue(new InstantCommand(() -> subsys.runMotor(kKrakenSpeed)));  //TODO: which climb speed is up/down?
            //stop climb
            raise.onFalse(new InstantCommand(() -> subsys.stop()));
            lower.onFalse(new InstantCommand(() -> subsys.stop())); 


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
