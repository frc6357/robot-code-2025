package frc.robot.utils;

public interface GenericController {
    /**
     * Method to return the mapped port that corresponds to the parameter
     * @param port The current port used for the default controller
     * @return The new mapped value that corresponds with the new controller
     */
    public int getMap(int port);
}
