package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.Animation;
import static frc.robot.Konstants.LightConstants.*;
import static frc.robot.Ports.LightsPorts.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants;
import frc.robot.utils.Duo;
import frc.robot.utils.ledAnimations.StaticColorAnimation;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.Timer;

public class SK25Lights implements Subsystem{
    SimpleWidget colorWidget;
    boolean isFinished;
    double time;

    ArrayList<Duo<Animation, Double>> animationQueue = new ArrayList<Duo<Animation, Double>>();
    ArrayList<Duo< Duo<Animation, Double>, Double>> effectQueue = new ArrayList<Duo< Duo<Animation, Double>, Double>>();

    private boolean isPresent;

    private CANdle m_candle;
    public SK25Lights(Optional<CANdle> candle){ // candle is assumed to be present because of subsystem init logic in RobotContainer
        m_candle = candle.get();
    }

    /**
     * Initializes the optional CANdle object with the an actual CANdle object. Useful for simplifying light commands
     * while keeping functionality for turning off lights in subsystem json
     */
    public void init()
    {
        
        CANdleConfiguration config = new CANdleConfiguration();
        
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.disableWhenLOS = true;
        config.statusLedOffWhenActive = true;
    
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        config.v5Enabled = false;
        
        m_candle.configAllSettings(config);

        m_candle.clearAnimation(1);
        this.setTeamColor();

        
    }
    
    /**
     * 
     * @param red The amount of Red to set, range is [0, 255]
     * @param green The amount of Green to set, range is [0, 255]
     * @param blue The amount of Blue to set, range is [0, 255]
     */
    public void setLight(int red, int green, int blue, int numLed){
         m_candle.setLEDs(red, green, blue, 0, 8, numLed);
    }

    /**
     * 
     * @param bright Brightness for all LED's controlled, range is [0.0, 1.0]
     */
    public void setBrightness(double bright){
        
        m_candle.configBrightnessScalar(bright);
    }

    public void RainbowAnimate(double brightness, double speed, int numLed, double duration){
        RainbowAnimation animation = new RainbowAnimation(brightness, speed, numLed, false, 8);
        addToQueue(animation, duration);
    }

    public void addToQueue(Animation anim, double duration) {
        animationQueue.add(Duo.of(anim, duration));
    }

    public void clearAnimate(){
        m_candle.clearAnimation(1);
    }

    public void clearQueue() {
        animationQueue.clear();
    }

    private boolean queueEmpty() {
        return animationQueue.isEmpty();
    }
    
    public void processQueue() {
        if (!queueEmpty()) {
            Duo<Animation, Double> currentAnim = getActiveAnimation();
            double elapsedTime = Timer.getFPGATimestamp() - time;  
    
            if (elapsedTime >= currentAnim.getSecond()) {
                m_candle.clearAnimation(1);  
                animationQueue.remove(0);  
            } else {
                m_candle.animate(currentAnim.getFirst(), 1);
            }
        }
    }
   


    public Duo<Animation, Double> getActiveAnimation() {
        if(queueEmpty()) {
            return null;
        }
        else{
            return animationQueue.get(0);
        }
    }

    public Duo< Duo<Animation, Double>, Double> getActiveAnimationWithTimestamp() {
        if(queueEmpty()) {
            return null;
        }
        else {
            return effectQueue.get(0);
        }
    }

    public Animation geAnimationFromQueue() { // Index 0 is considered the first animation in line
        return animationQueue.get(0).getFirst(); // getFirst references the Animation object in the Duo
    }

    public double getAnimationDurationFromQueue() {
        return animationQueue.get(0).getSecond(); // getSecond references the timestamp (double) object in the Duo
    }

    public boolean isFinished()
    {
        return isFinished;
    }

    public void setIsFinished(boolean finished){
        isFinished = finished;
    }

    public void setOrange(){
        StaticAnimate(255, 24, 0, kNumLedOnBot, Double.POSITIVE_INFINITY);
    }

    public void setGreen(){
        StaticAnimate(0, 255, 0, kNumLedOnBot, Double.POSITIVE_INFINITY);
    }

    public void setRed(){
        StaticAnimate(255, 0, 0, kNumLedOnBot, Double.POSITIVE_INFINITY);
    }

    public void setPurple(){
        StaticAnimate(128, 10, 128, kNumLedOnBot, Double.POSITIVE_INFINITY);
    }

    public void setTeamColor()
    {
        StaticAnimate(29, 168, 168, kNumLedOnBot, Double.POSITIVE_INFINITY);
    }

    public void setPartyMode(double duration)
    {
        RainbowAnimate(1.0, 1.0, kNumLedOnBot, duration);
    }

    public void strobeLightsGreenWhite(int r, int g, int b, int w)
    {
        StrobeAnimation animation = new StrobeAnimation(0, 255,0, 255, 100, kNumLedOnBot);
        m_candle.animate(animation, 1);
    }
    
    public void FlowAnimate(int r, int g, int b, double speed, int numLed, Direction direction, int offset)
    {

        ColorFlowAnimation animation = new ColorFlowAnimation( r,  g,  b,  0,  speed,  numLed,  direction, offset);
        m_candle.animate(animation, 1);
    }

    public void FireAnimate(double brightness, double speed, int numLed, double sparking, double cooling)
    {
        FireAnimation animation = new FireAnimation(brightness, speed, numLed, sparking, cooling, false, 8);
        m_candle.animate(animation, 1);
    }

    public void LarsonAnimate(int r, int g, int b, int numLed)
    {
        BounceMode mode = BounceMode.Center;
        LarsonAnimation animation = new LarsonAnimation(r, g, b, 0, 0.1, numLed, mode, numLed / 2, 8 );
        m_candle.animate(animation, 1);
    }

    public void RgbFadeAnimate(double brightness, double speed, int numLed)
    {
        RgbFadeAnimation animation = new RgbFadeAnimation(brightness, speed, numLed, 8);
        m_candle.animate(animation, 1);
    }

    public void SingleFadeAnimate(int r, int g, int b, double speed, int numLed)
    {
        SingleFadeAnimation animation = new SingleFadeAnimation(r, g, b, 0, speed, numLed, 8);
        m_candle.animate(animation, 1);
    }

    public void StrobeAnimate(int r, int g, int b, double speed, int numLed)
    {
        StrobeAnimation animation = new StrobeAnimation(r, g, b, 0, speed, numLed, 8);
        m_candle.animate(animation, 1);
    }

    public void TwinkleAnimate(int r, int g, int b, int numLed)
    {
        TwinklePercent percent = TwinklePercent.Percent100;
        TwinkleAnimation animation = new TwinkleAnimation(r, g, b, 0, 1.0, numLed, percent, 8);
        m_candle.animate(animation, 1);
    }

    public void StaticAnimate(int r, int g, int b, int numLed, double duration) 
    {
        StaticColorAnimation staticAnimation = new StaticColorAnimation(r, g, b, numLed);
        SingleFadeAnimation animation = staticAnimation.getAnimation();
        addToQueue(animation, duration);
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    @Override
    public void periodic()
    {
        time = Timer.getTimestamp();

        Duo<Duo<Animation, Double>, Double > currentAnimWithTimestamp = getActiveAnimationWithTimestamp(); 

            //awant to be able to set current anim with whatever timestamp it was whenever it wad first initialized
            if (currentAnimWithTimestamp != null) {
                double animationEndTime = currentAnimWithTimestamp.getSecond();
                if (time >= animationEndTime) {
                    // The current animation has finished, remove it from the queue
                    animationQueue.remove(0);
                }
            }

    } 

    public void testLEDs() {
        setLight(255, 0, 0, kNumLedOnBot);  
        Timer.delay(1);
        setLight(0, 255, 0, kNumLedOnBot);  
        Timer.delay(1);
        setLight(0, 0, 255, kNumLedOnBot);  
        Timer.delay(1);
        setLight(255, 255, 255, kNumLedOnBot); 
        Timer.delay(1) 
        clearAnimate();
    }
    
    testLEDs();

    }

