package frc.robot.utils.konstantLib.wrappers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SKTrigger
{
    public final GenericHID controller;
    public final Trigger button;
    public double axisReturnThreshold;
    public int port;
    INPUT_TYPE type;

    public enum INPUT_TYPE
    {
        AXIS,
        BUTTON,
        POV,
    }

    public SKTrigger(GenericHID controller, int port, INPUT_TYPE type)
    {
        this.controller = controller;
        this.type = type;
        this.port = port;
        this.axisReturnThreshold = 0.5;

        switch (type)
        {
            case AXIS:
                button = new Trigger(() -> controller.getRawAxis(port) >= axisReturnThreshold);
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

    /** Sets the axis threshold for an AXIS Trigger to return true or false.
     * AXIS returns true if the axis is greater than or equal to the newThreshold.
     * @param newThreshold The new threshold value to determine the return boolean of the 
     * AXIS trigger based on how far the AXIS is pressed. Value should be between 0.0 and 1.0.
     */
    public void setAxisReturnThreshold(double newThreshold)
    {
        axisReturnThreshold = newThreshold;
    }

    /** Gets the raw axis of the AXIS of the SKTrigger object.
     * @return The AXIS value from -1.0 to 1.0, returns 0.0 if the SKTrigger is not of type AXIS.
     */
    public double getRawControllerAxis()
    {
        switch(type)
        {
            case AXIS:
                return controller.getRawAxis(port);
            default:
                return 0.0;
        }
    }
}
