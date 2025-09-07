package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This is made for the 2022 season
 */
public class SubsystemControls
{

    private final boolean swerve;
    private final boolean lights;
    private final boolean elevator;
    private final boolean endEffector;
    private final boolean climb;
    private final boolean coralSubsystem;
    private final boolean vision;

     /**  
     * @param swerve
     *            indicates if the swerve subsystem is present and should be enabled
     * @param lights
     *            indicates if the lights subsystem is present and should be enabled
     */
    public SubsystemControls(
        @JsonProperty(required = true, value = "swerve")      boolean swerve,
        @JsonProperty(required = true, value = "lights")      boolean lights,
        @JsonProperty(required = true, value = "elevator")    boolean elevator,
        @JsonProperty(required = true, value = "endeffector") boolean endeffector,
        @JsonProperty(required = true, value = "climb") boolean climb,
        @JsonProperty(required = true, value = "coralSubsystem") boolean coralSubsystem,
        @JsonProperty(required = true, value = "vision")      boolean vision
    )
    {
        this.swerve = swerve;
        this.lights = lights;
        this.elevator = elevator;
        this.endEffector = endeffector;
        this.climb = climb;
        this.coralSubsystem = coralSubsystem;
        this.vision = vision;
    }


    /**
     * Returns true if the drive subsystem is indicated as present and should be enabled.
     * 
     * @return true if the drive subsystem is indicated as present and should be enabled; false
     *         otherwise
     */
    public boolean isSwervePresent()
    {
        return swerve;
    }
    public boolean isVisionPresent() {
        return vision;
    }
    public boolean isEndEffectorPresent()
    {
        return endEffector;
    } 
    public boolean isClimbPresent() 
    {
        return climb;
    }
    public boolean isCoralSubsystemPresent() 
    {
        return coralSubsystem;
    }
    public boolean isLightsPresent() {
        return lights;
    }
    public boolean isElevatorPresent() {
        return elevator;
    }
}
