package frc.robot.utils;

public abstract class SKControllerMapper {
    
    public static enum ControllerType{
        XBOX,
        GUITAR_HERO
    }
    
    public SKControllerMapper(ControllerType controller)
    {
        switch(controller){
            case XBOX:
                break;
            case GUITAR_HERO:
                break;
        }
    }
}
