package frc.robot.bindings;

import java.util.Optional;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Ports.OperatorPorts.kExampleButton;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ExampleBinder implements CommandBinder
{
    //create a subsystem toggleable by the json subsystems file
    Optional <ExampleSubsystem> myExampleSubsystem;

    //create the ExampleButton trigger object 
    Trigger ExampleButton;

    public ExampleBinder(Optional<ExampleSubsystem> SubsystemName)
    {
        this.myExampleSubsystem = SubsystemName;

        //tie the ExampleButton trigger the actual kExample button from Ports
        this.ExampleButton = kExampleButton.button;
    }

    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (myExampleSubsystem.isPresent())
        {
            ExampleSubsystem Subsystem = myExampleSubsystem.get();

            //run motor when pressed
            ExampleButton.onTrue(new InstantCommand(() -> Subsystem.runMotor()));
            //stop motor when released
            ExampleButton.onFalse(new ExampleCommand(Subsystem));
        }
    }

}
