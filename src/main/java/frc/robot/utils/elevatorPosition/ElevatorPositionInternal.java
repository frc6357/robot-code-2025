package frc.robot.utils.elevatorPosition;

/**
 * Generic class to set and read the height of an elevator in inches, with the lower point
 * starting at 0 and increasing towards the upper point. It does this using a motor that
 * contains an internal encoder, and can determine when it is at it's max point and zero
 * point using upper and lower sensors.
 */
public class ElevatorPositionInternal
{
    GenericElevatorMotor motor;

    /**
     * Enumerated Value that determines the motor type that is used for the elevator
     */

    public static enum HeightMotorType
    {
        /**
         * CAN Spark Flex motor
         */
        SparkFlex
    }

    /**
     * Creates a new elevator motor of the specified type
     * 
     * @param motorType
     *            Type of motor the elevator is using
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     * @param MinOutput
     *            Value for minimum output of the PID controller
     * @param MaxOutput
     *            Value for maximum output of the PID controller
     * @param lowerSensorID
     *            ID for digital input sensor that determines lower limit of elevator or -1 to
     *            indicate no switch is present
     * @param upperSensorID
     *            ID for digital input sensor that determines upper limit of elevator or -1 to
     *            indicate no switch is present
     */
    public ElevatorPositionInternal(HeightMotorType motorType, int CanID, double gearRatio, double Kp,
        double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput, int lowerSensorID, int upperSensorID)
    {
        switch (motorType)
        {
            case SparkFlex:
                motor = new SparkFlexElevator(CanID, gearRatio, Kp, Ki, Kd, Kiz, MinOutput, MaxOutput, lowerSensorID,
                    upperSensorID);
                break;
        }
    }

    /**
     * Creates a new elevator motor of the specified type
     * 
     * @param motorType
     *            Type of motor the elevator is using
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     * @param MinOutput
     *            Value for minimum output of the PID controller
     * @param MaxOutput
     *            Value for maximum output of the PID controller
     * @param lowerSensorID
     *            ID for digital input sensor that determines lower limit of elevator or -1 to
     *            indicate no switch is present
     */

    public ElevatorPositionInternal(HeightMotorType motorType, int CanID, double gearRatio, double Kp,
        double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput, int lowerSensorID)
    {
        switch (motorType)
        {
            case SparkFlex:
                motor = new SparkFlexElevator(CanID, gearRatio, Kp, Ki, Kd, Kiz, MinOutput, MaxOutput, lowerSensorID);
                break;
        }
    }

    /**
     * Creates a new elevator motor of the specified type
     * 
     * @param motorType
     *            Type of motor the elevator is using
     * @param CanID
     *            Can ID of the motor used
     * @param gearRatio
     *            Number of motor shaft rotations per output shaft rotations
     * @param Kp
     *            Value for proportional gain constant in PID controller
     * @param Ki
     *            Value for integral gain constant in PID controller
     * @param Kd
     *            Value for derivative gain constant in PID controller
     * @param Kiz
     *            Value for I Zone constant in PID controller
     * @param MinOutput
     *            Value for minimum output of the PID controller
     * @param MaxOutput
     *            Value for maximum output of the PID controller
     */

    public ElevatorPositionInternal(HeightMotorType motorType, int CanID, double gearRatio, double Kp,
        double Ki, double Kd, double Kiz, double MinOutput, double MaxOutput)
    {
        switch (motorType)
        {
            case SparkFlex:
                motor = new SparkFlexElevator(CanID, gearRatio, Kp, Ki, Kd, Kiz, MinOutput, MaxOutput);
                break;
        }
    }


    /*
     * Reset Encoder Stuff
     */


    /**
     * Resets position of encoder to 0.0.
     */
    public void resetEncoder()
    {
        motor.resetEncoder();
    }

    /**
     * Resets position of encoder to given height
     * 
     * @param height
     *          The desired height to reset the position to
     */
    public void resetEncoder(double height)
    {
        motor.resetEncoder(height);
    }
    

    /*
     * Follower Motor Stuff
     */


    /**
     * Adds a new motor that follows the actions of the lead motor
     * 
     * @param CanID
     *            CanID for the follower motor
     */

    public void addFollowerMotor(int CanID)
    {
        motor.addFollowerMotor(CanID);
    }


    /*
     * Lower Limit Switch Stuff
     */


    /**
     * @return Returns the value of digital input sensor that is used for location the
     *         lower limit of the elevator.
     */
    public boolean isLowerReached()
    {
        return motor.isLowerReached();
    }

    /**
     * @return Returns true if the lower sensor is present
     */
    public boolean isLowerAvailable()
    {
        return motor.isLowerAvailable();
    }


    /*
     * Upper Limit Switch Stuff
     */


    /**
     * @return Returns the value of digital input sensor that is used for locating the
     *         upper limit of the elevator.
     */
    public boolean isUpperReached()
    {
        return motor.isUpperReached();
    }

    /**
     * @return Returns true if the upper sensor is present
     */
    public boolean isUpperAvailable()
    {
        return motor.isUpperAvailable();
    }


    /*
     * Stop. Simple Enough.
     */


    /**
     * Stops motor movement. Motor can be moved again by calling set without having to
     * re-enable the motor.
     */
    public void stop()
    {
        motor.stop();
    }


    /*
     * Getting Left & Right Target Position Stuff
     */


    /**
     * @return Returns the current setpoint that the elevator is attempting to reach
     */
    public double getLeftTargetPosition()
    {
        return motor.getLeftTargetPosition();
    }

    /**
     * @return Returns the current setpoint that the elevator is attempting to reach
     */
    public double getRightTargetPosition()
    {
        return motor.getRightTargetPosition();
    }


    /*
     * Getting Left & Right Position Stuff
     */


    /**
     * @return Returns the height that the elevator is currently at
     */
    public double getLeftPosition()
    {
        return motor.getLeftPosition();
    }

    /**
     * @return Returns the height that the elevator is currently at
     */
    public double getRightPosition()
    {
        return motor.getRightPosition();
    }


    /*
     * Getting Left & Right Target Position Stuff
     */


    /**
     * Sets the height of the elevator to specified inches, starting at 0 at the lower point
     * and increasing towards upper point
     * 
     * @param height
     *            Inches to which the elevator should be set
     */
    public void setTargetPosition(double height)
    {
        motor.setTargetPosition(height);
    }


    /*
     * More Limit Switch Stuff
     */



    /**
     * 
     * Checks both the upper and lower sensor, if they are present, to determine if they
     * have been reached. If so it will stop the motor, and reset the encoder if it has
     * reached the bottom sensor.
     */

    public void checkLimitSensors()
    {
        if (isLowerAvailable() && isLowerReached())
        {
            resetEncoder();
            stop();
        }

        if (isUpperAvailable() && isUpperReached())
        {
            stop();
        }
    }

    public void testInit(){
        motor.testInit();
    }

    public void periodic()
    {
        motor.periodic();
    }

    public GenericElevatorMotor getMotor()
    {
        return motor;
    }
}
