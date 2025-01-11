package frc.robot.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * A class that uses the slew rate limiter to limit the rate of change of the input.
 */
public class SlewRateFilter implements Filter
{

    private final SlewRateLimiter slewFilter;
    private double                reversed;
    private double                deadband;
    private double                gain;
    private double                slope;

    /**
     * Creates a filter with a max specified change rate. This prevents an input from
     * changing too quickly.
     * 
     * @param filterRate
     *            The max rate at which the input can change in units per second
     * @param deadband
     *            Used as an area around the zero of the controller inputs. Should be a
     *            positive value. This is used to stop natural controller drift and small
     *            accidental inputs. Deadband is measured in percent of raw axis value.
     * @param reversed
     *            Whether the sign of the input should be inverted
     */
    public SlewRateFilter(double filterRate, double deadband, boolean reversed)
    {
        this(filterRate, 1.0, deadband, reversed);
    }

    /**
     * Creates a filter with a max specified change rate. This prevents an input from
     * changing too quickly.
     * 
     * @param filterRate
     *            The max rate at which the input can change in units per second
     * @param gain
     *            The max value that should be output by the axis after filtering. This is
     *            multiplied the raw axis to get the final value
     * @param deadband
     *            Used as an area around the zero of the controller inputs. Should be a
     *            positive value. This is used to stop natural controller drift and small
     *            accidental inputs.
     * @param reversed
     *            Whether the sign of the input should be inverted
     */
    public SlewRateFilter(double filterRate, double gain, double deadband, boolean reversed)
    {
        slewFilter = new SlewRateLimiter(filterRate);
        this.reversed = reversed ? -1.0 : 1.0;
        this.deadband = deadband;
        this.gain = gain;

        slope = 1 / (1 - this.deadband);
    }

    /**
     * Filters the given input value per the rules of this Filter, and returns the
     * filtered value.
     * 
     * @param rawAxis
     *            The actual value being returned by the raw data
     * @return The filtered data to be passed to the motor
     */
    public double filter(double rawAxis)
    {
        if (Math.abs(rawAxis) < deadband)
        {
            return slewFilter.calculate(0.0);
        }
        else
        {
            return reversed * slewFilter.calculate(gain * (rawAxis - deadband) * slope);
        }
    }

    /**
     * Sets whether the sign of input should be inverted
     * 
     * @param reversed
     *            Whether or not should the output sign should be reversed.
     */
    public void setInversion(boolean reversed)
    {
        this.reversed = reversed ? -1.0 : 1.0;
    }
}
