package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAlgaeGrab;
import static frc.robot.Ports.OperatorPorts.kAlgaeRelease;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeCommand;
import frc.robot.subsystems.AlgaePickup;

public class AlgaeBinder implements CommandBinder
{
    //create a subsystem toggleable by the json subsystems file
    Optional <AlgaePickup> m_AlgaeSubsystem;

    //create the ExampleButton trigger object 
    Trigger Release;
    Trigger Grab;


    public AlgaeBinder(Optional<AlgaePickup> SubsystemName)
    {
        this.m_AlgaeSubsystem = SubsystemName;

        //tie the ExampleButton trigger the actual kExample button from Ports
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
            Release.onTrue(new AlgaeCommand(Subsystem));

            //lowers and then grabs
            Grab.onTrue(new AlgaeCommand(Subsystem));
        }
    }

}
