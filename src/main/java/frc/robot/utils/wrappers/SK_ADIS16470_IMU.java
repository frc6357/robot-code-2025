package frc.robot.utils.wrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;

/**
 * An ADIS16470IMU that can be used to measure rotations, acceleration, temprature, etc.
 */
public class SK_ADIS16470_IMU extends ADIS16470_IMU
{

    public SK_ADIS16470_IMU()
    {
        super();
    }

    /**
     * @param yaw_axis
     *            The axis that measures the yaw
     * @param pitch_axis
     *            The axis that measures the pitch
     * @param roll_axis
     *            The axis that measures the roll
     * @param port
     *            The SPI Port the gyro is plugged into
     * @param cal_time
     *            Calibration time
     */
    public SK_ADIS16470_IMU(final IMUAxis yaw_axis, final IMUAxis pitch_axis, final IMUAxis roll_axis, SPI.Port port, CalibrationTime cal_time)
    {
        super(yaw_axis, roll_axis, pitch_axis, port, cal_time);
    }

    /**
     * Uses the gyro angle to find Rotation2d
     * 
     * @return calculated Rotation2d (CCW+)
     */
    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getAngle(getYawAxis()));
    }
}