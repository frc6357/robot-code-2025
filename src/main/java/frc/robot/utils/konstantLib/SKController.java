package frc.robot.utils.konstantLib;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.utils.konstantLib.wrappers.SKCommandXboxTrigger.XBOX_INPUT_TYPE.*;
import static frc.robot.utils.konstantLib.wrappers.SKTrigger.INPUT_TYPE.*;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utils.files.Elastic;
import frc.robot.utils.files.Elastic.Notification.NotificationLevel;
import frc.robot.utils.konstantLib.filters.FilteredAxis;
import frc.robot.utils.konstantLib.filters.FilteredXboxController;
import frc.robot.utils.konstantLib.wrappers.SKCommandXboxTrigger;
import frc.robot.utils.konstantLib.wrappers.SKTrigger;

 
public class SKController 
{
    private int port;
    private ControllerType type;
    private FilteredXboxController xboxController;
    private GenericHID hIDController;

    public enum ControllerType
    {
        XBOX,
        HID,
        GUITAR_HERO,
        KEYBOARD,
        GCN;
    }

    public enum AXIS_TYPE
    {
        LEFT_Y(kLeftY.value),
        LEFT_X(kLeftX.value),
        RIGHT_Y(kRightY.value),
        RIGHT_X(kRightX.value),
        LEFT_TRIGGER(kLeftTrigger.value),
        RIGHT_TRIGGER(kRightTrigger.value);

        public int axisPort;

        AXIS_TYPE(int axisPort)
        {
            this.axisPort = axisPort;
        }
    }

    public SKController(ControllerType type, int port)
    {
        this.type = type;
        this.port = port;

        switch(type){

            case XBOX:
                //initialize the filtered xbox controller
                //FilteredXboxController class is a wrapper class which extends CommandXboxController and allows a filter to be applied.
                xboxController = new FilteredXboxController(port);

                //Create and send an elastic notificaiton (of importance level INFO) that the controller is connected, and include its port and type.
                Elastic.Notification commandXboxNotification = new Elastic.Notification(NotificationLevel.INFO, 
                "Controller connceted to port " + String.valueOf(port), 
                "A Controller of type CommandXboxController has been recognized and is active.");
                //send and display the notification for 8.0 seconds
                Elastic.sendNotification(commandXboxNotification.withDisplaySeconds(8.0));

                break;

            case HID:
                //assign the command xbox controller to the typeless controller object 
                hIDController = new GenericHID(port);

                //Create and send an elastic notificaiton (of importance level INFO) that the controller is connected, and include its port and type.
                Elastic.Notification genericHIDNotification = new Elastic.Notification(NotificationLevel.INFO, 
                "Controller connceted to port " + String.valueOf(port), 
                "A Controller of type GenericHID has been recognized and is active.");
                //send and display the notification for 8.0 seconds
                Elastic.sendNotification(genericHIDNotification.withDisplaySeconds(8.0));

                break;

            case GUITAR_HERO:
                //make new guitar hero controller
                hIDController = new GenericHID(port);

                //Create and send an elastic notificaiton (of importance level INFO) that the controller is connected, and include its port and type.
                Elastic.Notification guitarHeroNotification = new Elastic.Notification(NotificationLevel.INFO, 
                "Controller connceted to port " + String.valueOf(port), 
                "A Controller of type GenericHID has been recognized and is active.");
                //send and display the notification for 8.0 seconds
                Elastic.sendNotification(guitarHeroNotification.withDisplaySeconds(8.0));

                break;

            case KEYBOARD:
                //make a new key input controller
                hIDController = new GenericHID(port);

                //Create and send an elastic notificaiton (of importance level INFO) that the controller is connected, and include its port and type.
                Elastic.Notification keyboardNotification = new Elastic.Notification(NotificationLevel.INFO, 
                "Controller connceted to port " + String.valueOf(port), 
                "A Controller of type GenericHID has been recognized and is active.");
                //send and display the notification for 8.0 seconds
                Elastic.sendNotification(keyboardNotification.withDisplaySeconds(8.0));

                break;

            case GCN:
                //make new key input controller with GCN methods controller
                hIDController = new GenericHID(port);

                //Create and send an elastic notificaiton (of importance level INFO) that the controller is connected, and include its port and type.
                Elastic.Notification gameCubeNotification = new Elastic.Notification(NotificationLevel.INFO, 
                "Controller connceted to port " + String.valueOf(port), 
                "A Controller of type GenericHID has been recognized and is active.");
                //send and display the notification for 8.0 seconds
                Elastic.sendNotification(gameCubeNotification.withDisplaySeconds(8.0));

                break;
        }
    }

    /**Gets the controller port.*/
    public int getPort()
    {
        return port;
    }

    /**If port is not at least zero or at most five, no new port is assigned. */
    public void setPort(int newPort)
    {
        //The controller port must be a value of or between 0 and 5.
        if (newPort >= 0 && newPort <= 5)
        {
            port = newPort;
        }
    }
    
    /*Gets the underlying GenericHID object of the SKController*/
    public GenericHID getUnderlyingHIDController()
    {
        switch(type)
        {
            case XBOX:
                return xboxController.getHID();
            default:
                return hIDController;
        }
    }

    /*Gets the underlying FilteredXboxController (extends CommandXboxController) object of 
    the SKController*/
    public FilteredXboxController getUnderlyingXboxController() throws Exception
    {
        switch(type)
        {
            case XBOX:
                return xboxController;
            default:
                throw new Exception("Attempted to call getUnderlyingXboxController on a controller not of enum type XBOX");
        } 
    }

    /** Returns the specified filtered axis of the SKController using the AXIS_TYPE enum types.
     * If that balue is a joystick input, the value will be between -1.0 and 1.0. If it is a trigger,
     * the value will be between 0.0 and 1.0.
     * @param axisType The axis value to return, enum types are listed at the top of the SKControlle class
     * in the AXIS_TYPE enum class (ex:LEFT_Y).
     * @return The FilteredAxis value of the controller port, which contains a Supplier<Double> type
     * value.*/
    public FilteredAxis getRawSKContorllerAxis(AXIS_TYPE axisType)
    {
        switch(type)
        {
            case XBOX:
                return new FilteredAxis(() -> xboxController.getRawAxis(axisType.axisPort));
            default:
                return new FilteredAxis(() -> hIDController.getRawAxis(axisType.axisPort));
        }
    }

    /**Maps the SKTrigger object to the A button value of the controller. */
    public SKTrigger mapA()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kA.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kA.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kA.value, BUTTON); //lower 1 button
            case KEYBOARD:
                return new SKTrigger(hIDController, kA.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kA.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kA.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the B button value of the controller. */
    public SKTrigger mapB()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kB.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kB.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kB.value, BUTTON); //upper 1
            case KEYBOARD:
                return new SKTrigger(hIDController, kB.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kB.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kB.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the X button value of the controller. */
    public SKTrigger mapX()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kX.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kX.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kX.value, BUTTON); //upper 2
            case KEYBOARD:
                return new SKTrigger(hIDController, kX.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kX.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kX.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the Y button value of the controller. */
    public SKTrigger mapY()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kY.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kY.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kY.value, BUTTON); //upper 3
            case KEYBOARD:
                return new SKTrigger(hIDController, kY.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kY.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kY.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the left shoulder button value of the controller. */
    public SKTrigger mapLeftShoulderButton()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftBumper.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kLeftBumper.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kLeftBumper.value, BUTTON); //lower 2
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftBumper.value, BUTTON);
            case GCN:
                return null;   //GCN dosn't have a left shoulder button
            default:
                return new SKTrigger(hIDController, kLeftBumper.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the right shoulder button value of the controller. */
    public SKTrigger mapRightShoulderButton()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightBumper.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kRightBumper.value, BUTTON);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kRightBumper.value, BUTTON); //lower 3
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightBumper.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kRightBumper.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kRightBumper.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the start/menu button value of the controller. */
    public SKTrigger mapStart()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kStart.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            case GUITAR_HERO:
                return null;  //Guitar Hero dosn't have a start button equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the select/back button value of the controller. */
    public SKTrigger mapSelect()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kBack.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a select button equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
            case GCN:
                return null; //GCN dosn't have a select button
            default:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
        }
    }

    /**Maps the SKTrigger object to the DPad's up button value of the controller. */
    public SKTrigger mapUpDPad()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), 0, POV, XBOX_POV);
            case HID:
                return new SKTrigger(hIDController, 0, POV);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, 0, POV); //up joystick
            case KEYBOARD:
                return new SKTrigger(hIDController, 0, POV);
            case GCN:
                return new SKTrigger(hIDController, 0, POV);
            default:
                return new SKTrigger(hIDController, 0, POV);
        }
    }

    /**Maps the SKTrigger object to the DPad's down button value of the controller. */
    public SKTrigger mapDownDPad()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), 180, POV, XBOX_POV);
            case HID:
                return new SKTrigger(hIDController, 180, POV);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, 180, POV); //down joystick
            case KEYBOARD:
                return new SKTrigger(hIDController, 180, POV);
            case GCN:
                return new SKTrigger(hIDController, 180, POV);
            default:
                return new SKTrigger(hIDController, 180, POV);
        }
    }

    /**Maps the SKTrigger object to the DPad's left button value of the controller. */
    public SKTrigger mapLeftDPad()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), 270, POV, XBOX_POV);
            case HID:
                return new SKTrigger(hIDController, 270, POV);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, 270, POV); //left joystick
            case KEYBOARD:
                return new SKTrigger(hIDController, 270, POV);
            case GCN:
                return new SKTrigger(hIDController, 270, POV);
            default:
                return new SKTrigger(hIDController, 270, POV);
        }
    }

    /**Maps the SKTrigger object to the DPad's right button value of the controller. */
    public SKTrigger mapRightDPad()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), 90, POV, XBOX_POV);
            case HID:
                return new SKTrigger(hIDController, 90, POV);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, 90, POV); //right joystick
            case KEYBOARD:
                return new SKTrigger(hIDController, 90, POV);
            case GCN:
                return new SKTrigger(hIDController, 90, POV);
            default:
                return new SKTrigger(hIDController, 90, POV);
        }
    }

    /**Maps the SKTrigger object to the right trigger axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapRightTrigger()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightTrigger.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a right trigger axis equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the left trigger axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapLeftTrigger()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftTrigger.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a left trigger axis equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the left joystick X axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapLeftJoystickX()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftX.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a left joystick axis equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the left joystick Y axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapLeftJoystickY()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftY.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftY.value, AXIS);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kLeftY.value, AXIS);  //strum bar, -1.0 if up, 1.0 if down, no inbetween
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftY.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftY.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftY.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the right joystick X axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapRightJoystickX()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightX.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a right joystick axis equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the right joystick Y axis value of the controller.
    * <p>
    * AXIS returns a boolean, not a numerical value!
    *For a number, use controller.getRawSKControllerAxis(AXIS_TYPE axisPort) instead.
    <p>
    To Set the value which determines if the Trigger returns true or false, use 
    setAxisReturnThreshold(double newThreshold) on the SKTrigger object.*/
    public SKTrigger mapRightJoystickY()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightY.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            case GUITAR_HERO:
                return null; //Guitar Hero dosn't have a right joystick axis equivelant
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the left joystick press button value of the controller. */
    public SKTrigger mapLeftJoystickPress()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftStick.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftStick.value, AXIS);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kLeftStick.value, AXIS);   //hero power
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftStick.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftStick.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftStick.value, AXIS);
        }
    }

    /**Maps the SKTrigger object to the right joystick press button value of the controller. */
    public SKTrigger mapRightJoystickPress()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightStick.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightStick.value, AXIS);
            case GUITAR_HERO:
                return new SKTrigger(hIDController, kRightStick.value, AXIS);  //pause
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightStick.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightStick.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightStick.value, AXIS);
        }
    }


    

    //FOR GUITAR HERO:

    //A button is lower 1
    //B button is upper 1
    //X button is upper 2
    //Y button is upper 3
    //left bumper is lower 2
    //right bumper is lower 3
    //right trigger is
    //left trigger is 
    //left stick X is 
    //left stick Y is strum bar, -1.0 if up, 1.0 if down, no inbetween
    //right stick X is
    //right stick Y is
    //start is 
    //back is 
    //up is joystick up
    //left is koystick left
    //right is joystick right
    //down is joystick down
    //left stick press is hero power
    //right stick press is pause

    //motion input, whammy bar, capture and reset dont have known mappable values (more testing required)
    
    //try kXInputGuitar, kXInputGuitar2, and kXInputGuitar3.
}
