package frc.robot.utils.filters;

/**
 * This class is a filter that has a deadband for it's return. If a value is filtered and
 * in the deadband, it returns zero.
 * 
 * By default, scale is readjusted to have a slope from y=0 to y=1 starting at x=deadband
 * and ending at x=1.
 * 
 * The gain value can be modified to alter the slope (The gain is a multiplier on the
 * scope after the scope calculation is performed).
 */
public class DeadbandFilter implements Filter
{
    /**
     * The deadbanding for the input, equal to distance from zero.
     */
    private double deadband;

    /**
     * The slope m of the y=mx line to get careful control.
     */
    private double slope;

    /**
     * The gain is the multiplier on the slope that would otherwise provide a line from
     * x=deadband,y=0 to x=1,y=1.
     */
    private double gain;

    /**
     * Constructs a new DeadbandFilter with a deadband of the given width.
     * 
     * @param deadband
     *            The width of the deadbanding zone, equivalent to the distance from zero
     */
    public DeadbandFilter(double deadband)
    {
        gain = 1;
        setDeadband(deadband);
    }

    /**
     * Constructs a new DeadbandFilter with a deadband of the given width and the provided
     * gain.
     * 
     * @param deadband
     *            The width of the deadbanding zone, equivalent to the distance from zero
     * @param gain
     *            The gain value used to amplify the slope of values calculated above the
     *            deadband
     */
    public DeadbandFilter(double deadband, double gain)
    {
        setDeadband(deadband);
        this.gain = gain;
    }

    /**
     * Filters the raw input for a deadbanding zone. If the given value is inside the
     * deadband, zero is returned. Otherwise, a scale is implemented and slope is
     * readjusted for careful control.
     * 
     * @param rawAxis
     *            The data to be passed in, from -1 to 1
     * @return The value after the DeadbandFilter has been applied
     */
    @Override
    public double filter(double rawAxis)
    {
        if (Math.abs(rawAxis) < deadband)
        {
            return 0.0;
        }
        else
        {
            return (slope * (rawAxis - deadband)) * gain;
        }
    }

    /**
     * Sets the deadband of the filter to the given value, with length equal to distance
     * to zero.
     * 
     * @param deadband
     *            The new deadband value for this DeadbandFilter
     */
    public void setDeadband(double deadband)
    {
        this.deadband = deadband;
        slope = 1 / (1 - deadband);
    }

    /**
     * Sets the gain value used to amplify the slope of values calculated above the
     * deadband.
     * 
     * @param gain
     *            The new gain to adjust the filtered value in this DeadbandFilter
     */
    public void setGain(double gain)
    {
        this.gain = gain;
    }
}
