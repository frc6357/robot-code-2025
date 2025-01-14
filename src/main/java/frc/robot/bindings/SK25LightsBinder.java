package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLightsToTealButton;
import static frc.robot.Ports.OperatorPorts.kPartyModeButton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25Lights;

public class SK25LightsBinder implements CommandBinder
{
    //create a lights subsystem toggleable by the json subsystems file
    Optional <SK25Lights> m_lights;

    //create the trigger objects
    Trigger PartyMode;
    Trigger LightsToTeal;

    public SK25LightsBinder(Optional<SK25Lights> lights)
    {
        this.m_lights = lights;

        //tie the triggers to their respective buttons
        this.LightsToTeal = kLightsToTealButton.button;
        this.PartyMode = kPartyModeButton.button;
    }

    public void bindButtons()
    {
        //if the subsytem is present in the json subsystems file
        if (m_lights.isPresent())
        {
            SK25Lights lights = m_lights.get();

            //set lights to teal (team color)
            LightsToTeal.onTrue(new InstantCommand(() -> lights.setTeamColor()));
            //set lights to party mode (rainbow animate)
            PartyMode.onTrue(new InstantCommand(() -> lights.setPartyMode()));
        }
    }

}
