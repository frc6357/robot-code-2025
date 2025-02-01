package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAlgaeGrab;
import static frc.robot.Ports.OperatorPorts.kAlgaeRelease;
import static frc.robot.Ports.OperatorPorts.kAlgaeRaise;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlgaeStopCommand;
import frc.robot.commands.AlgaeReturnCommand;
import frc.robot.commands.AlgaeReleaseCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AlgaePickup;

public class AlgaeBinder implements CommandBinder
{
    //create a subsystem toggleable by the json subsystems file
    Optional <AlgaePickup> m_AlgaeSubsystem;

    //create the ExampleButton trigger object 
    Trigger Raise;
    Trigger Grab;
    Trigger Release;


    public AlgaeBinder(Optional<AlgaePickup> SubsystemName)
    {
        this.m_AlgaeSubsystem = SubsystemName;

        //tie the ExampleButton trigger the actual kExample button from Ports
        this.Raise = kAlgaeRaise.button;
        this.Release = kAlgaeRelease.button;
        this.Grab =  kAlgaeGrab.button;
    }

    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (m_AlgaeSubsystem.isPresent())
        {
            AlgaePickup Subsystem = m_AlgaeSubsystem.get();

            //run motor when pressed
            //releases then raises

            //lowers and then grabs
            Release.onTrue(new AlgaeReleaseCommand(Subsystem));
            Raise.onTrue(new AlgaeReturnCommand(Subsystem));
            Grab.onTrue(new AlgaeCommand(Subsystem));
            Grab.onFalse(new AlgaeStopCommand(Subsystem));
            Release.onFalse(new AlgaeStopCommand(Subsystem));
        }
    }

}
