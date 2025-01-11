package frc.robot.utils.filters;

/**
 * Creates a cubic curve on the filter 0.0 correlates to 0.0, and 1.0 correlates to
 * "coeff" and -1.0 correlates to "-coeff". The transfer function is
 * 
 * output = coefficient * (input^3)
 */
public class CubicFilter implements Filter
{
    /**
     * Coefficient of gain to multiply by.
     */
    private double gain;

    /**
     * Constructs a new ExponentialFilter with the given gain coefficient.
     * 
     * @param gain
     *            The coefficient of the cubic function for this ExponentialFilter. This
     *            can be positive or negative, allowing the joystick value to be inverted
     *            at the same time as the filter is applied.
     */
    public CubicFilter(double gain)
    {
        this.gain = gain;
    }

    /**
     * Filters the input using a scaled cubic transfer function.
     * 
     * @param rawAxis
     *            The data to be filtered, from -1 to 1
     * @return The cubic relation of that data
     */
    @Override
    public double filter(double rawAxis)
    {
        return gain * Math.pow(rawAxis, 3);
    }

    /**
     * Sets the coefficient of the cubic function of this ExponentialFilter.
     * 
     * @param g
     *            The new coefficient for this ExponentialFilter, which must be greater
     *            than zero
     */
    public void setGain(double g)
    {
        gain = Math.abs(g);
    }
}
