package frc.robot.utils.filters;

import java.util.function.Supplier;

public class FilteredAxis
{

    private Supplier<Double> axis;
    private Filter filter;

    public FilteredAxis(Supplier<Double> supplier, Filter filter)
    {
        this.axis = supplier;
        this.filter = filter;
    }

    public FilteredAxis(Supplier<Double> supplier)
    {
        this(supplier, null);
    }

    /**
     * This method returns a filtered number from the requested axis. The Filter used is
     * determined from the static mapping provided via setFilter.
     * 
     * @return The filtered value from the axis
     */
    public double getFilteredAxis()
    {
        if (filter != null)
        {
            return filter.filter(axis.get());
        }
        else
        {
            return axis.get();
        }
    }

    /**
     * Sets a pair between a axis and a Filter for user's general control.
     * 
     * @param axis
     *            The Joystick axis to pair with a Filter
     * @param f
     *            The Filter to pair with a axis
     */
    public void setFilter(Filter f)
    {
        filter = f;
    }

    /**
     * Returns the Filter object currently associated with a given joystick axis or null
     * if no Filter is currently in use.
     *
     * @return The current filter for the axis
     */
    public Filter getFilter()
    {
        return filter;
    }
}
