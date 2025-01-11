package frc.robot.utils;

/**
 * A class that contains the CAN ID and which CAN bus a device is on.
 */
public final class CANPort
{
    /** The CAN ID of the device */
    public final Integer ID;
    /** The name of the CAN bus that the device is on */
    public final String  bus;

    /**
     * Creates a CANPort that contains all the information required to instantiate a CAN
     * device.
     * 
     * @param ID
     *            The CAN ID of the device
     * @param busName
     *            The name of the CAN bus (put "" if on the default Roborio CAN bus)
     */
    public CANPort(Integer ID, String busName)
    {
        this.ID = ID;
        this.bus = busName;
    }
}
