package frc.robot.utils.elevatorPosition;

/**
 * Generic class to create different types of motors to be used to find elevator heights.
 */
public abstract class GenericElevatorMotor
{
    /**
     * Resets position of encoder to 0.0
     */
    public abstract void resetEncoder();

    /**
     * Resets position of encoder to given height
     * 
     * @param height
     *          The desired height to reset the position to
     */
    public abstract void resetEncoder(double height);

    /**
     * Adds a new motor that follows the actions of the lead motor
     * 
     * @param CanID
     *            CanID for the follower motor
     */
    public abstract void addFollowerMotor(int CanID);

    /**
     * @return Returns the value of the sensor that is used for locating the lower limit
     *         of the elevator.
     */
    public abstract boolean isLowerReached();

    /**
     * @return Returns true if the lower sensor is present
     */
    public abstract boolean isLowerAvailable();

    /**
     * @return Returns the value of the sensor that is used for locating the upper limit
     *         of the elevator.
     */
    public abstract boolean isUpperReached();

    /**
     * @return Returns true if the lower sensor is present
     */
    public abstract boolean isUpperAvailable();

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to
     * re-enable the motor.
     */
    public abstract void stop();

    /**
     * 
     * @return Returns the current setpoint the elevator is attempting to reach
     */
    public abstract double getLeftTargetPosition();

    /**
     * 
     * @return Returns the current setpoint the elevator is attempting to reach
     */
    public abstract double getRightTargetPosition();

    /**
     * 
     * @return Returns the current height the elevator is at in this moment of time
     */
    public abstract double getLeftPosition();

    /**
     * 
     * @return Returns the current height the elevator is at in this moment of time
     */
    public abstract double getRightPosition();

    /**
     * Sets the height of the elevator to specified inches
     * 
     * @param height
     *            Height to which the elevator should be set
     */
    public abstract void setTargetPosition(double height);

    public abstract void testInit();

    public abstract void periodic();
}