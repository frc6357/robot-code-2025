package frc.robot.subsystems.vision;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.Trio;

import frc.robot.subsystems.SKSwerve;

public class SK25Vision extends SubsystemBase implements NTSendable {
    public final Limelight backLL = new Limelight(VisionConfig.BACK_CONFIG);
    public final Limelight frontLL = new Limelight(VisionConfig.FRONT_CONFIG);
    private SKSwerve m_swerve;


    //TODO: Add logging/telemetry for both limelights

    public final Limelight[] allLimelights = {frontLL, backLL}; // List of all limelights
    public final Limelight[] poseLimelights = {frontLL, backLL}; // Limelights specifically used for estimating pose

    private final DecimalFormat df = new DecimalFormat();

    public boolean isIntegrating = false; // Boolean statign whether or not limelight poses are being integrated with actual

    // Creates an ArrayList to store estimated vision poses during autonomous 
    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();


    public SK25Vision(Optional<SKSwerve> m_swerve) {
        this.m_swerve = m_swerve.get();
        df.setMaximumFractionDigits(2);

        /* Limelight startup configurator*/
        for(Limelight ll : allLimelights) {
            ll.setLEDMode(false); // Turns off LED lights on startup
        }
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        SmartDashboard.putData("Vision",
        new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Vision");

                builder.addStringProperty("BackLLStatus", () -> backLL.getLogStatus(), null);
                builder.addStringProperty("FrontLLStatus", () -> frontLL.getLogStatus(), null);
            }
        });
    }

    @Override
    public void periodic() {
        double yaw = m_swerve.getRotation().getDegrees();

        for(Limelight ll : poseLimelights) {
            ll.setRobotOrientation(yaw); // Periodically updates limelight orientation based on robot facing

        /* Autonomous pose updater */
        if(DriverStation.isAutonomousEnabled() && ll.targetInView()) {
            Pose3d botPose3d = ll.getRawPose3d(); // Gets the 3D pose of the limelight on the robot using the position offsets
            Pose2d megaPose2d = ll.getMegaPose2d(); // Gets the estimated pose of the robot using MegaTag2 objects
            double timestamp = ll.getRawPoseTimestamp(); // Timestamp of when the pose was collected

            Pose2d estimatedRobotPose = // Uses limelight (effectively swerve) rotation value instead of estimated rotation value
                new Pose2d(megaPose2d.getTranslation(), botPose3d.toPose2d().getRotation()); 
            
            autonPoses.add(Trio.of(botPose3d, estimatedRobotPose, timestamp)); // Adds both poses into the auto pose feeder
            }
        }

        try {
            isIntegrating = false;
            if(DriverStation.isTeleopEnabled()) {
                Limelight bestLL = getBestLimelight();
                for(Limelight ll : poseLimelights) {
                    if(ll.getName() == bestLL.getName()) {
                        // Add vision pose input
                    }
                    else {
                        ll.sendInvalidStatus("Rejected: Not best Limelight");
                    }
                    // A limelight's integrating status is determined by if it's received a valid status to integrate its pose
                    isIntegrating |= ll.getConfig().isIntegrating(); 
                }
            }
        }
        catch(Exception e) {
            DriverStation.reportWarning("Vision: Attempted to access nonexistant vision pose!", false);
        }

    }

    public Limelight getBestLimelight() {
        Limelight bestLimelight = frontLL; // Default limelight to consider "best". Front limelight often has best view of tag
        double bestScore = 0;
        for(Limelight LL : poseLimelights) {
            double score = 0;
            score += LL.getTagCountInView();
            score += LL.getTargetSize(); // Range: 1-100

            if(score > bestScore) {
                bestScore = score;
                bestLimelight = LL;
            }
        }

        return bestLimelight;
    }




}
