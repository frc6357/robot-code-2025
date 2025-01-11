package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKTrigger
{
    public final GenericHID controller;
    public final Trigger button;

    public enum INPUT_TYPE
    {
        AXIS,
        BUTTON,
        POV,
        ROCKBAND;
    }

    public SKTrigger(GenericHID controller, int port, INPUT_TYPE type)
    {
        this.controller = controller;

        switch (type)
        {
            case AXIS:
                button = new Trigger(() -> controller.getRawAxis(port) > 0.5);
                break;

            case BUTTON:
                button = new JoystickButton(controller, port);
                break;

            case POV:
                button = new POVButton(controller, port);
                break;

            default:
                button = null;
        }
    }
}
