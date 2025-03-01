package frc.robot.utils.ledAnimations;

import com.ctre.phoenix.led.SingleFadeAnimation;


public class StaticColorAnimation{
    private SingleFadeAnimation animation;

    public StaticColorAnimation(int r, int g, int b, int w, int numLed) {
        animation = new SingleFadeAnimation(r, g, b, w, 0, numLed, 0);
    }
    public StaticColorAnimation(int r, int g, int b, int numLed) {
        animation = new SingleFadeAnimation(r, g, b, 0, 0, numLed, 0);
    }

    public SingleFadeAnimation getAnimation() {
        return animation;
    }
}
