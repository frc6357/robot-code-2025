package frc.robot.utils.wrappers;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.CANPort;

/**
 * Java class that wraps WPI_CANCoder to allow offsetDegrees as parameter that changes
 * position to an offset position.
 */

public class SK_CANCoder extends WPI_CANCoder
{

    private double offsetDegrees;

    public SK_CANCoder(CANPort deviceNumber, double offsetDegrees)
    {
        super(deviceNumber.ID, deviceNumber.bus);

        this.offsetDegrees = offsetDegrees;
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.unitString = "deg";
        config.sensorDirection = false; //Counter clockwise positive
        config.magnetOffsetDegrees = this.offsetDegrees;
        config.sensorCoefficient = 360.0 / 4096;

        this.configAllSettings(config);
    }

    /**
     * Uses the measured angle to find Rotation2d
     * 
     * @return calculated Rotation2d (CCW+)
     */
    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getAbsolutePosition());
    }
}
