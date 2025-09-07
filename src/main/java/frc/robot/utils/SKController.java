package frc.robot.utils;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
import static frc.robot.utils.SKCommandXboxTrigger.XBOX_INPUT_TYPE.*;
//import edu.wpi.first.wpilibj.GenericHID.HIDType;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utils.files.Elastic;
import frc.robot.utils.files.Elastic.Notification.NotificationLevel;
import frc.robot.utils.filters.FilteredXboxController;


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

    public int getPort()
    {
        return port;
    }

    //if port is not at least one or at most five, no new port is assigned
    public void setPort(int newPort)
    {
        if (newPort >= 1 && newPort <= 5)
        {
            port = newPort;
        }
    }
    
    //gets the underlying hid controller
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

    //only works for xbox
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
                return null;
            default:
                return new SKTrigger(hIDController, kLeftBumper.value, BUTTON);
        }
    }

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

    public SKTrigger mapStart()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kStart.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            case GCN:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
            default:
                return new SKTrigger(hIDController, kStart.value, BUTTON);
        }
    }

    public SKTrigger mapSelect()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kBack.value, BUTTON, XBOX_BUTTON);
            case HID:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
            case GCN:
                return null;
            default:
                return new SKTrigger(hIDController, kBack.value, BUTTON);
        }
    }

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

    public SKTrigger mapRightTrigger()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightTrigger.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightTrigger.value, AXIS);
        }
    }

    public SKTrigger mapLeftTrigger()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftTrigger.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftTrigger.value, AXIS);
        }
    }

    public SKTrigger mapLeftJoystickX()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kLeftX.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
            default:
                return new SKTrigger(hIDController, kLeftX.value, AXIS);
        }
    }

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

    public SKTrigger mapRightJoystickX()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightX.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightX.value, AXIS);
        }
    }

    public SKTrigger mapRightJoystickY()
    {
        switch(type)
        {
            case XBOX: 
                return new SKCommandXboxTrigger(xboxController, xboxController.getHID(), kRightY.value, AXIS, XBOX_AXIS);
            case HID:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            case GUITAR_HERO:
                return null;
            case KEYBOARD:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            case GCN:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
            default:
                return new SKTrigger(hIDController, kRightY.value, AXIS);
        }
    }

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
