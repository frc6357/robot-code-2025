package frc.robot.subsystems.superclasses;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.ElevatorConstants.ElevatorPosition;

public abstract class Elevator extends SubsystemBase
{
    /**
     * Set the elevator height to a specific height using pre-set values.
     * 
     * @param height
     *            Enum that specifies which height (in shaft rotations) you want the elevator to be set at.
     */
    public abstract void setTargetHeight(ElevatorPosition height);

    /**
     * @return Returns the height (In shaft rotations) that the elevator is currently at.
     */
    public abstract double getEncoderPosition();

    /**
     * @return Returns the current target position that the elevator is attempting to reach.
     */
    public abstract double getTargetPosition();

    /**
     * 
     * @return Returns true if the elevator has reached its current set point.
     */
    public abstract boolean isAtTargetPosition();
}