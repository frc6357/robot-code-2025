package frc.robot.utils;


public abstract class SKControllerCreator {
    
    public static enum ControllerType{
        XBOX,
        HID,
        GUITAR_HERO,
        GCN
    }
    
    public SKControllerCreator(ControllerType controller)
    {
        switch(controller){
            case XBOX:
                //public final CommandXboxController commandXboxCont = new CommandXboxController(0);
                break;
            case HID:
                break;
            case GUITAR_HERO:
                break;
            case GCN:
                break;
        }
    }
}
