package frc.robot.utils;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Implements an Encoder that scales the values on the encoder. This is used to convert
 * encoder pulses to wheel rotations (to abstract away the number of pulses per
 * revolution).
 */
public class ScaledEncoder extends Encoder
{
    /**
     * Indicates the pulses per rotation for this ScaledEncoder.
     */
    private final int pulsesPerRotation;

    /**
     * Constructs a new ScaledEncoder which extends an Encoder with knowledge of the
     * pulses per revolution and diameter of the drive wheels.
     * 
     * @param channelA
     *            The digital input ID for channel A
     * @param channelB
     *            The digital input ID for channel B
     * @param reverseDirection
     *            Sets if the encoder direction is reversed
     * @param pulses
     *            The number of pulses per revolution for the encoder
     * @param diameter
     *            Diameter of the drivetrain wheels
     */
    public ScaledEncoder(int channelA, int channelB, boolean reverseDirection, int pulses,
        double diameter)
    {
        super(channelA, channelB, reverseDirection);
        pulsesPerRotation = pulses;
        setDistancePerPulse((diameter * Math.PI) / pulses);
    }

    /**
     * Constructs a new ScaledEncoder which extends an Encoder with knowledge of the
     * pulses per revolution and diameter of the drive wheels. This constructor assumes
     * the direction is not reversed.
     * 
     * @param channelA
     *            The digital input ID for channel A
     * @param channelB
     *            The digital input ID for channel B
     * @param pulses
     *            The number of pulses per revolution for the encoder
     * @param diameter
     *            Diameter of the drivetrain wheels
     */
    public ScaledEncoder(int channelA, int channelB, int pulses, double diameter)
    {
        super(channelA, channelB);
        pulsesPerRotation = pulses;
        setDistancePerPulse((diameter * Math.PI) / pulses);
    }

    /**
     * Returns the amount of degrees the wheel has turned.
     * 
     * @return The amount of degrees the wheel has turned
     */
    public double getAngleDegrees()
    {
        return (get() * 360) / pulsesPerRotation;
    }

    /**
     * Returns the amount of radians the wheel has turned.
     * 
     * @return The amount of radians the wheel has turned
     */
    public double getAngleRadians()
    {
        return (get() * 2 * Math.PI) / pulsesPerRotation;
    }

    /**
     * Returns the amount of rotations the wheel has turned
     * 
     * @return The amount of rotations the wheel has turned
     */
    public double getRotations()
    {
        return get() / pulsesPerRotation;
    }
}
