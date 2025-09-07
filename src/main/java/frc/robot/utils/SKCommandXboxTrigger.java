package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.filters.FilteredXboxController;

public class SKCommandXboxTrigger extends SKTrigger{
    
    public final Trigger button;
    public final FilteredXboxController xbox;
    public final GenericHID controller;
    public final int port;
    public final INPUT_TYPE type;

    public enum XBOX_INPUT_TYPE
    {
        XBOX_AXIS,
        XBOX_BUTTON,
        XBOX_POV;
    }

    public SKCommandXboxTrigger(FilteredXboxController xbox, GenericHID controller, int port, INPUT_TYPE type, XBOX_INPUT_TYPE xType)
    {
        super(controller, port, type);
        this.controller = controller;
        this.port = port;
        this.type = type;
        this.xbox = xbox;
        
        switch (xType)
        {
            case XBOX_AXIS:
                button = new Trigger(() -> xbox.getFilteredAxis(port) > 0.5);
                break;

            case XBOX_BUTTON:
                button = new JoystickButton(xbox.getHID(), port);
                break;

            case XBOX_POV:
                button = new POVButton(xbox.getHID(), port);
                break;

            default:
                button = null;
        }

    }

    public SKTrigger getUnderlyingSKTrigger()
    {
        return new SKTrigger(controller, port, type);
    }
}
