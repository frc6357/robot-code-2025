package frc.robot.subsystems.vision;

import java.text.DecimalFormat;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.Trio;
import frc.robot.RobotContainer;

public class SK25Vision extends SubsystemBase {
    public final Limelight backLL = new Limelight(VisionConfig.BACK_CONFIG);
    public final Limelight frontLL = new Limelight(VisionConfig.FRONT_CONFIG);

    //TODO: Add logging/telemetry for both limelights

    public final Limelight[] allLimelights = {frontLL, backLL}; // List of all limelights
    public final Limelight[] poseLimelights = {frontLL, backLL}; // Limelights specifically used for estimating pose

    private final DecimalFormat df = new DecimalFormat();

    // Creates an ArrayList to store estimated vision poses during autonomous 
    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();


    public SK25Vision() {
        df.setMaximumFractionDigits(2);

        /* Limelight startup configurator*/
        for(Limelight ll : allLimelights) {
            ll.setLEDMode(false); // Turns off LED lights on startup
        }
    }

    @Override
    public void periodic() {
        double yaw = RobotContainer.drivetrain.getRotation().getDegrees();
    }
}
