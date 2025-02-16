package frc.robot.subsystems.superclasses;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;

public abstract class Elevator extends SubsystemBase
{
    /*
     * Sets Both Target Heights
     */


    /**
     * Set the elevator height to a specific height using pre-set values.
     * 
     * @param height
     *            Enum that specifies which height you want the elevator to be set at
     */
    public abstract void setTargetHeight(ElevatorPosition height);


    /*
     * Set Target Heights
     */


    /**
     * Sets the right elevator motor to the specified height in inches
     * 
     * @param height
     *            Height to which the elevator is set to (inches)
     */
    //public abstract void setRightTargetHeight(double height);

    /**
     * Sets the left elevator motor to the specified height in inches
     * 
     * @param height
     *            Height to which the elevator is set to (inches)
     */
    //public abstract void setLeftTargetHeight(double height);


    /*
     * Get Positions (Encoder)
     */


    /**
     * @return Returns the height that the elevator is currently at
     */
    public abstract double getEncoderPosition();


    /*
     * Get Target Positions
     */


    /**
     * @return Returns the current setpoint that the elevator is attempting to reach
     */
    public abstract double getLeftTargetPosition();

    /**
     * @return Returns the current setpoint that the elevator is attempting to reach
     */
    public abstract double getRightTargetPosition();


    /*
     * Is At Target Position
     */


    /**
     * 
     * @return Returns true if the elevator has reached its current set point.
     */
    public abstract boolean isRightAtTargetPosition();

    /**
     * 
     * @return Returns true if the elevator has reached its current set point.
     */
    public abstract boolean isLeftAtTargetPosition();
}