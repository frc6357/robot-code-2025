package frc.robot.utils.filters;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class filters Joystick inputs based on a series of provided Filter objects.
 * 
 * Values read from an axis using getFilteredAxis() will be the result of the Filter being
 * applied to the raw axis value if a Filter for that axis has been provided.
 */
public class FilteredXboxController extends CommandXboxController
{
    private static final int MAX_AXES = 16;

    private Filter[] filters;

    /**
     * Constructs a new FilteredJoystick for the given Joystick port.
     * 
     * @param joystickNumber
     *            The port of the Joystick this FilteredJoystick
     */
    public FilteredXboxController(int joystickNumber)
    {
        super(joystickNumber);
        filters = new Filter[MAX_AXES];
    }

    /**
     * This method returns a filtered number from the requested axis. The Filter used is
     * determined from the static mapping provided via setFilter.
     * 
     * @param axis
     *            The Joystick axis to get the data from
     * @return The filtered value from the Joystick axis
     */
    public double getFilteredAxis(int axis)
    {
        if (filters[axis] != null)
        {
            return filters[axis].filter(getRawAxis(axis));
        }
        else
        {
            return getRawAxis(axis);
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
    public void setFilter(int axis, Filter f)
    {
        filters[axis] = f;
    }

    /**
     * Returns the Filter object currently associated with a given joystick axis or null
     * if no Filter is currently in use.
     *
     * @param axis
     *            The joystick axis for which the current paired Filter should be returned
     * @return The current filter for the given Joystick axis
     */
    public Filter getFilter(int axis)
    {
        return filters[axis];
    }
}
