package frc.robot.subsystems;

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
import static frc.robot.Konstants.LightConstants.*;
import static frc.robot.Ports.LightsPorts.*;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class SK25Lights{
    Optional<CANdle> candle;
    SimpleWidget colorWidget;
    boolean isFinished;
    
    public SK25Lights(){
        candle = Optional.empty();
    }

    /**
     * Initializes the optional CANdle object with the an actual CANdle object. Useful for simplifying light commands
     * while keeping functionality for turning off lights in subsystem json
     */
    public void init()
    {
        candle = Optional.of(new CANdle(kCANdle.ID)); // creates a new CANdle with ID kCANdle's ID from ports
        
        CANdleConfiguration config = new CANdleConfiguration();
        
        
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.disableWhenLOS = true;
        config.statusLedOffWhenActive = true;
    
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        config.v5Enabled = false;
        
        candle.get().configAllSettings(config);

        candle.get().clearAnimation(1);
        this.setTeamColor();

        
    }
    
    /**
     * 
     * @param red The amount of Red to set, range is [0, 255]
     * @param green The amount of Green to set, range is [0, 255]
     * @param blue The amount of Blue to set, range is [0, 255]
     */
    public void setLight(int red, int green, int blue, int numLed){
        if(candle.isPresent()){ candle.get().setLEDs(red, green, blue, 0, 8, numLed);} 
    }

    /**
     * 
     * @param bright Brightness for all LED's controlled, range is [0.0, 1.0]
     */
    public void setBrightness(double bright){
        
        if(candle.isPresent()){ candle.get().configBrightnessScalar(bright);}
    }

    public void RainbowAnimate(double brightness, double speed, int numLed){
        RainbowAnimation animation = new RainbowAnimation(brightness, speed, numLed, false, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void clearAnimate(){
        if(candle.isPresent()){ candle.get().clearAnimation(1);}
    }

    public boolean isFinished()
    {
        return isFinished;
    }

    public void setIsFinished(boolean finished){
        isFinished = finished;
    }

    public void setOrange(){
        setLight(255, 24, 0, numLedOnBot);
    }

    public void setGreen(){
        setLight(0, 255, 0, numLedOnBot);
    }

    public void setRed(){
        setLight(255,0,0,numLedOnBot);
    }

    public void setPurple(){
        setLight( 128, 10, 128, numLedOnBot);
    }

    public void setTeamColor()
    {
        setLight(0, 250, 150, numLedOnBot);
    }

    public void setPartyMode()
    {
        RainbowAnimate(1.0, 1.0, numLedOnBot);
    }
    
    
    public void FlowAnimate(int r, int g, int b, double speed, int numLed, Direction direction, int offset){

        ColorFlowAnimation animation = new ColorFlowAnimation( r,  g,  b,  0,  speed,  numLed,  direction, offset);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void FireAnimate(double brightness, double speed, int numLed, double sparking, double cooling){
        FireAnimation animation = new FireAnimation(brightness, speed, numLed, sparking, cooling, false, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void LarsonAnimate(int r, int g, int b, int numLed){
        BounceMode mode = BounceMode.Center;
        LarsonAnimation animation = new LarsonAnimation(r, g, b, 0, 0.1, numLed, mode, numLed / 2, 8 );
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void RgbFadeAnimate(double brightness, double speed, int numLed){
        RgbFadeAnimation animation = new RgbFadeAnimation(brightness, speed, numLed, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void SingleFadeAnimate(int r, int g, int b, double speed, int numLed){
        SingleFadeAnimation animation = new SingleFadeAnimation(r, g, b, 0, speed, numLed, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void StrobeAnimate(int r, int g, int b, double speed, int numLed){
        StrobeAnimation animation = new StrobeAnimation(r, g, b, 0, speed, numLed, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    public void TwinkleAnimate(int r, int g, int b, int numLed){
        TwinklePercent percent = TwinklePercent.Percent100;
        TwinkleAnimation animation = new TwinkleAnimation(r, g, b, 0, 1.0, numLed, percent, 8);
        if(candle.isPresent()){ candle.get().animate(animation, 1);}
    }

    //occurs every 20 miliseconds, usually not tied to a command, binder, etc...
    public void periodic()
    {
    } 

    public void testInit()
    {
    }
    
    public void testPeriodic()
    {
    }
    
}

