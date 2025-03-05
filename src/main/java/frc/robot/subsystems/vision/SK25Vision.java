package frc.robot.subsystems.vision;

import static frc.robot.Konstants.VisionConstants.kAprilTagPipeline;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import frc.robot.utils.Trio;
import frc.robot.utils.Field;
import frc.robot.RobotContainer;
import frc.robot.TunerConstants;
import frc.robot.subsystems.SKSwerve;

public class SK25Vision extends SubsystemBase implements NTSendable {
    public final Limelight backLL = new Limelight(VisionConfig.BACK_CONFIG);
    public final Limelight frontLL = new Limelight(VisionConfig.FRONT_CONFIG);
    private SKSwerve m_swerve;

    private static final int[] blueReefTagIDs = {17, 18, 19, 20, 21, 22};
    private static final int[] redReefTagIDs = {6, 7, 8, 9, 10, 11};

    //TODO: Add logging/telemetry for both limelights

    public final Limelight[] allLimelights = {frontLL, backLL}; // List of all limelights
    public final Limelight[] poseLimelights = {frontLL, backLL}; // Limelights specifically used for estimating pose

    private final DecimalFormat df = new DecimalFormat();

    public boolean isIntegrating = false; // Boolean statign whether or not limelight poses are being integrated with actual

    // Creates an ArrayList to store estimated vision poses during autonomous 
    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();


    public SK25Vision(Optional<SKSwerve> m_swerveContainer) {
        this.m_swerve = m_swerveContainer.get();
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
                builder.addStringProperty("ResetPoseToVisionStatus", () -> resetPoseToVisionLog, null);
            }
        });
    }

    public static final class AlignTranslationWithPose extends CommandConfig {
        private AlignTranslationWithPose() {
            configKp(0.2);
            configTolerance(0.01);
            configMaxOutput(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.6);
            configError(0.3);
            configPipelineIndex(kAprilTagPipeline);
            configLimelight(RobotContainer.m_vision.frontLL);
        }

        public static AlignTranslationWithPose getConfig() {
            return new AlignTranslationWithPose();
        }
    }

    public static final class AlignRotationWithPose extends CommandConfig {
        private AlignRotationWithPose() {
            configKp(0);
        }
    }

    // TODO: Make new pipelines for both limelights that only detects each alliance's reef tags
    public static final class AlignToReefTag extends CommandConfig {
        private AlignToReefTag() {
            configKp(0.02);
            configTolerance(0.01);
            configMaxOutput(TunerConstants.MaxSpeed * 0.6);
            configError(0.3);
            configPipelineIndex(kAprilTagPipeline);
            configLimelight(RobotContainer.m_vision.frontLL);
        }

        public static AlignToReefTag getConfig() {
            return new AlignToReefTag();
        }
    }

    public static final class DriveToPose extends CommandConfig {
        private DriveToPose() {
            
        }
    }

    @Override
    public void periodic() {
        double yaw = m_swerve.getRotation().getDegrees();

        for(Limelight ll : poseLimelights) {
        // Used for MEGATAG 2
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
                        addFilteredLimelightInput(bestLL);
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
            DriverStation.reportWarning("Vision: Attempted to access nonexistent vision pose!", false);
        }

        

    }

    public static boolean reefTargetSeen(Limelight ll) { 
        RawFiducial[] tags = ll.getRawFiducial(); // Assuming we're on blue alliance
        int[] targetIDs = blueReefTagIDs;

        if(Field.isRed()) { // Check to make sure if Red Alliance
            targetIDs = redReefTagIDs;
        }

        /*Since all reef tags are sequential, just check upper
        and lower bound of reef tag sequence. If seen tag's id
        is part of the targeted reef tags, return true. */ 
        for(RawFiducial tag : tags) {
            if((tag.id >= targetIDs[0]) && (tag.id <= targetIDs[5])) { 
                return true;
            }
        }

        // If none of the iterated tags on this limelight match our alliance's reef tags:
        return false;
    }

    public enum FIELD_ELEMENT { // We don't really care how close the other tags are (processor, barge)
        REEF
    }

    /**
     * Finds the closest AprilTag for a specific field element for your alliance.
     * SURROUND THIS IN TRY/CATCH! It will return null if no AprilTags of specific target are seen!
     * @param ll The limelight to reed tags from
     * @param element The field element's tag(s) to target
     * @return The closest matching tag found.
     */
    public static RawFiducial getClosestTargetFiducial(Limelight ll, FIELD_ELEMENT element) {
        RawFiducial[] tags = ll.getRawFiducial();
        ArrayList<RawFiducial> goodTags = new ArrayList<RawFiducial>();
        int[] targetIDs = {};

        if(tags.length == 0) {
            //TODO: make this report somewhere else
            DriverStation.reportWarning("Vision: Cannot find target tag; no tags seen!", false);
        }

        switch(element) {
            case REEF:
                targetIDs = blueReefTagIDs;
                if(Field.isRed()) {
                    targetIDs = redReefTagIDs;
                }

                // This specific logic of sequential id checking is only applicable
                // for the reef since the tags are in order and there are exactly 6
                for(RawFiducial tag : tags) {
                    if((tag.id >= targetIDs[0]) && (tag.id <= targetIDs[5])) { 
                        goodTags.add(tag);
                    }
                }
                break;
        }

        if(goodTags.isEmpty()) {
            //TODO: Make this report somewhere else
            DriverStation.reportWarning("Vision: No target tags found for " + element, false);
            return null;
        }

        RawFiducial closestTag = null;
        double closestTagDistance = 9999999; // Add a couple 9s for good measure
        for(RawFiducial tag : goodTags) {
            if(tag.distToRobot < closestTagDistance) {
                closestTagDistance = tag.distToRobot;
                closestTag = tag;
            }
        }

        return closestTag;
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

    public void forcePoseToVision() {
        Limelight ll = backLL;

        m_swerve.resetPose(ll.getMegaPose2d());
    }

    public void autonResetPoseToVision() {
        boolean reject = true;
        boolean firstSuccess = false;
        double batchSize = 5;
        /* Starting at the most recent auton pose estimation, analyze the next [batchSize]
        poses and see if they can be succesfully added to the robot's pose estimator
        */ 
        for (int i = autonPoses.size() - 1; i > (autonPoses.size() - batchSize) - 1; i--) {
            Trio<Pose3d, Pose2d, Double> poseInfo = autonPoses.get(i);
            boolean success =
                    resetPoseToVision(
                            true, poseInfo.getFirst(), poseInfo.getSecond(), poseInfo.getThird());
            if (success) {
                if (i == autonPoses.size() - 1) {
                    firstSuccess = true;
                }
                reject = false;
                // Print somewhere: "AutonResetPoseToVision succeeded on " + (autonPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            // Print somewhere
            //         "AutonResetPoseToVision failed after "
            //                 + batchSize
            //                 + " of "
            //                 + autonPoses.size()
            //                 + " possible tries");
            
            // Flash LEDs Red?
        } else {
            if (firstSuccess) {
                // Flash LEDs Green?
            } else {
                // Flash LEDs Orange?
            }
        }
    }

    public void resetPoseToVision() { // Calls resetPoseToVision by passing in Limelight measurements
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    private String resetPoseToVisionLog; // Provides an updatable string for smartdashboard
    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
        boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            Pose2d botpose = botpose3D.toPose2d();
            // Pose2d robotPose = m_swerve.getRobotPose(); // TODO: Add telemetry for pose before and after integrating vision
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25 // Robot pose is floating
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) {     
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || BAD POSE");
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) {
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || OUT OF FIELD");
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || IN AIR");
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                        resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || TILTED");
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;
            m_swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            m_swerve.addVisionMeasurement(integratedPose, poseTimestamp);
            // robotPose = m_swerve.getRobotPose(); // get updated pose
            resetPoseToVisionLog = ("ResetPoseToVision: SUCCESS");
            return true;
        }
        return false; // target not in view
    }

    private void addFilteredLimelightInput(Limelight LL) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (LL.targetInView()) {
            boolean multiTags = LL.multipleTagsInView();
            double timeStamp = LL.getRawPoseTimestamp();
            double targetSize = LL.getTargetSize();
            Pose3d botpose3D = LL.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = LL.getMegaPose2d();
            RawFiducial[] tags = LL.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = m_swerve.getVelocity(true);

            // distance from current pose to vision estimated pose
            double poseDifference =
                    m_swerve.getRobotPose().getTranslation().getDistance(botpose.getTranslation());

            /* rejections */

            // reject pose if individual tag ambiguity is too high
            LL.setTagStatus("");
            for (RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2) {
                    highestAmbiguity = tag.ambiguity;
                } else if (tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // log ambiguities
                LL.setTagStatus(LL.getTagStatus() + "Tag " + tag.id + ": " + tag.ambiguity);
                // ambiguity rejection check
                if (tag.ambiguity > 0.9) {
                    LL.sendInvalidStatus("ambiguity rejection");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                LL.sendInvalidStatus("bound rejection");
                return;
            } else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
                // reject if we are rotating more than 0.5 rad/s
                LL.sendInvalidStatus("rotation rejection");
                return;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                LL.sendInvalidStatus("height rejection");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                LL.sendInvalidStatus("roll/pitch rejection");
                return;
            } else if (targetSize <= 0.025) {
                LL.sendInvalidStatus("size rejection");
                return;
            }
            /* integrations */

            // if almost stationary and extremely close to tag
            else if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                LL.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } 
            // If multiple tags detected
            else if (multiTags && targetSize > 0.05) {
                LL.sendValidStatus("Multi integration");
                xyStds = 0.25;
                degStds = 8;
                if (targetSize > 0.09) { // If larger tag size
                    LL.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                    degStds = 0.1;
                }
            }
            // If tag is very close, loosen up pose strictness
            else if (targetSize > 0.8 && poseDifference < 0.5) {
                LL.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 16;
            } 
            // If tag is moderately close but pose difference is small
            else if (targetSize > 0.1 && poseDifference < 0.3) {
                LL.sendValidStatus("Proximity integration");
                xyStds = 2.0;
                degStds = 999999;
            } 
            // If inaccuracy (ambiguity) is low and target size is ok
            else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                LL.sendValidStatus("Stable integration");
                xyStds = 0.5;
                degStds = 999999;
            } 
            else {
                LL.sendInvalidStatus(
                        "catch rejection: "
                                + poseDifference
                                + " poseDiff");
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
                degStds = 15;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());

            addVisionMeasurementWithStdDevs(
                integratedPose, 
                timeStamp, 
                VecBuilder.fill(
                    VisionConfig.VISION_STD_DEV_X,
                    VisionConfig.VISION_STD_DEV_Y,
                    VisionConfig.VISION_STD_DEV_THETA));
        } 
        else {
            LL.setTagStatus("no tags");
            LL.sendInvalidStatus("no tag found rejection");
        }
    }

    private void addVisionMeasurementWithStdDevs(Pose2d integratedPose, double timeStamp, Vector<N3>stdDevs) {
        m_swerve.setVisionMeasurementStdDevs(stdDevs);

        m_swerve.addVisionMeasurement(integratedPose, timeStamp);
    }
    private void addVisionMeasurementWithStdDevs(Pose2d integratedPose, double timeStamp, double stdDevX, double stdDevY, double stdDevTheta) {
        Vector<N3> stdDevs = VecBuilder.fill(
            stdDevX,
            stdDevY,
            stdDevTheta
        );
        m_swerve.setVisionMeasurementStdDevs(stdDevs);
        m_swerve.addVisionMeasurement(integratedPose, timeStamp);
    }

    /** If at least one limelight has an accurate pose */
    public boolean hasAccuratePose() {
        for (Limelight limelight : poseLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    public static class CommandConfig {
        public double kp = 0;
        public double ki = 0;
        public double kd = 0;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        public void configKp(double kp) {
            this.kp = kp;
        }

        public void configKi(double ki) {
            this.ki = ki;
        }

        public void configKd(double kd) {
            this.kd = kd;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public void configAlignCommand(CommandConfig alignCommand) {
            this.alignCommand = alignCommand;
        }

        public CommandConfig() {}
    }
}
