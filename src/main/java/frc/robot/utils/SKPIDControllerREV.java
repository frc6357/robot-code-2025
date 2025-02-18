package frc.robot.utils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

public class SKPIDControllerREV extends SparkClosedLoopController{
    
    public SKPIDControllerREV()
    {
        super();

    }

    public SKPIDControllerREV(double P, double I, double D)
    {
        super();
    }

    public void applyAllPID(double currentPos, double P, double I, double D, double currentLimitAmps)
    {
        this.configs.setP(P).set(I).set(D).setCurrentLimit(currentLimitAmps);
        this.setReference(currentPos, ControlType.kMAXMotionPositionControl);
    }
}
