package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This is made for the 2022 season
 */
public class SubsystemControls
{

    private final boolean swerve;
    private final boolean example;
    private final boolean lights;

     /**  
     *@param theswerve
     *            indicates if the swerve subsystem is present and should be enabled
     * @param swerve
     *            indicates if the swerve subsystem is present and should be enabled
     * @param example
     *            indicates if the example subsystem is present and should be enabled
     * @param lights
     *            indicates if the lights subsystem is present and should be enabled
     */
    public SubsystemControls(
        @JsonProperty(required = true, value = "swerve")      boolean swerve,
        @JsonProperty(required = true, value = "example")      boolean example,
        @JsonProperty(required = true, value = "lights")      boolean lights
    )
    {
        this.swerve = swerve;
        this.example = example;
        this.lights = lights;
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
    public boolean isExamplePresent()
    {
        return example;
    }
    public boolean isLightsPresent()
    {
        return lights;
    }
}
