package frc.robot.utils.konstantLib.wrappers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.konstantLib.filters.FilteredXboxController;

public class SKCommandXboxTrigger extends SKTrigger{
    
    public final Trigger button;
    public final FilteredXboxController xbox;
    public final GenericHID controller;
    public final int port;
    public final INPUT_TYPE type;
    public double returnThreshold;
    XBOX_INPUT_TYPE xType;

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
        this.axisReturnThreshold = 0.5;
        this.port = port;
        this.type = type;
        this.xType = xType;
        this.xbox = xbox;
        
        switch (xType)
        {
            case XBOX_AXIS:
                button = new Trigger(() -> xbox.getFilteredAxis(port) > axisReturnThreshold);
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

    /**Gets the underlying SKTrigger object from the SKCommandTrigger object. */
    public SKTrigger getUnderlyingSKTrigger()
    {
        return new SKTrigger(controller, port, type);
    }

    /** Sets the axis threshold for an AXIS Trigger to return true or false.
     * AXIS returns true if the axis is greater than or equal to the newThreshold.
     * @param newThreshold The new threshold value to determine the return boolean of the 
     * AXIS trigger based on how far the AXIS is pressed. Value should be between 0.0 and 1.0.
     */
    @Override
    public void setAxisReturnThreshold(double newThreshold)
    {
        axisReturnThreshold = newThreshold;
    }

    /** Gets the raw axis of the AXIS of the SKTrigger object.
     * @return The AXIS value from -1.0 to 1.0, returns 0.0 if the SKTrigger is not of type AXIS.
     */
    @Override
    public double getRawControllerAxis()
    {
        switch(xType)
        {
            case XBOX_AXIS:
                return controller.getRawAxis(port);
            default:
                return 0.0;
        }
    }
}
