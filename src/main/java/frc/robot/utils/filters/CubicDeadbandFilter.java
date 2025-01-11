package frc.robot.utils.filters;

/**
 * CubicDeadbandFilter can filter a value through a cubic relation, but also applies a
 * deadband to the incoming value.
 */
public class CubicDeadbandFilter implements Filter
{
    /**
     * Coefficient of gain to multiply by.
     */
    private double coefficient;

    /**
     * Deadband value for the controller to avoid drift.
     */
    private double deadband;

    /**
     * Used to set the output value range to [-gain,gain].
     */
    private double gain = 1.0;

    /**
     * A value by which the input will be multiplied; Will have a value of 1.0 or -1.0,
     * depending on whether the joystick input values need to be reversed.
     */
    private double reverseFilter;

    /**
     * Constructs a new CubicDeadband filter with the given characteristics.
     * 
     * @param coefficient
     *            The coefficient of the cubic function. This can be positive or negative,
     *            allowing the joystick value to be inverted at the same time as the
     *            filter is applied. This number should be less than 1.
     * @param deadband
     *            Used as an area around the zero of the controller inputs. Should be a
     *            positive value. This is used to stop natural controller drift and small
     *            accidental inputs.
     * @param reverseInput
     *            Used to flip the joystick inputs to the opposite sign if required.
     */
    public CubicDeadbandFilter(double coefficient, double deadband, boolean reverseInput)
    {
        this.coefficient = coefficient;
        this.deadband = deadband;
        this.gain = 1.0;
        reverseFilter = (reverseInput ? -1.0 : 1.0);
    }

    /**
     * Constructs a new CubicDeadband filter with the given characteristics.
     * 
     * @param coefficient
     *            The coefficient of the cubic function. This can be positive or negative,
     *            allowing the joystick value to be inverted at the same time as the
     *            filter is applied. This number should be less than 1.
     * @param deadband
     *            Used as an area around the zero of the controller inputs. Should be a
     *            positive value. This is used to stop natural controller drift and small
     *            accidental inputs.
     * @param gain
     *            Used to set the output value range to [-gain,gain].
     * @param reverseInput
     *            Used to flip the joystick inputs to the opposite sign if required.
     */
    public CubicDeadbandFilter(double coefficient, double deadband, double gain,
        boolean reverseInput)
    {
        this.coefficient = coefficient;
        this.deadband = deadband;
        this.gain = gain;
        reverseFilter = (reverseInput ? -1.0 : 1.0);
    }

    /**
     * Filters the input using a scaled cubic transfer function.
     * 
     * @param rawAxis
     *            The data to be read in, from -1 to 1
     * @return The cubic relation of that data
     */

    @Override
    public double filter(double rawAxis)
    {
        // Used to set up the calculations for the inputs outside the deadband
        double filteredInput = (Math.abs(rawAxis) - deadband) * Math.signum(rawAxis);
        filteredInput *= reverseFilter;
        double c = (1 - (coefficient * Math.pow((1 - deadband), 3)) / (1 - deadband));

        // If it's within the deadband, it sets the input to zero
        if (Math.abs(rawAxis) < deadband)
        {
            return 0.0;
        }
        else
        {
            // Calculate the cubic output for the given coefficient and deadband
            return (coefficient * Math.pow(filteredInput, 3) + c * filteredInput) * gain;
        }

    }

    /**
     * Sets the coefficient of the cubic function.
     * 
     * @param newC
     *            The coefficient, which must be between zero and one
     */
    public void setCoef(double newC)
    {
        coefficient = Math.abs(newC);
    }

    /**
     * Sets the gain of the cubic function
     * 
     * @param newG
     *            The gain must be between -1 and 1
     */
    public void setGain(double newG)
    {
        gain = Math.abs(newG);
    }

    /**
     * Sets the deadband of the for the joystick input.
     * 
     * @param newD
     *            The new value used for the deadband for the controller
     */
    public void setDeadband(double newD)
    {
        deadband = Math.abs(newD);
    }
}
