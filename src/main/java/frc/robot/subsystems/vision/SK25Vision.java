package frc.robot.subsystems.vision;

import static frc.robot.Konstants.VisionConstants.kAprilTagPipeline;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;

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
import frc.robot.utils.vision.Limelight.IMUMode;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import frc.robot.utils.Trio;
import frc.robot.utils.Field;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Konstants.TunerConstants;
import frc.robot.subsystems.SKSwerve;

public class SK25Vision extends SubsystemBase implements NTSendable {
    public final Limelight backLL = new Limelight(VisionConfig.BACK_CONFIG); // limelight-alpha
    public final Limelight frontLL = new Limelight(VisionConfig.FRONT_CONFIG); // limelight-beta
    private SKSwerve m_swerve;

    private static final int[] blueReefTagIDs = {17, 18, 19, 20, 21, 22};
    private static final int[] redReefTagIDs = {6, 7, 8, 9, 10, 11};

    public final Limelight[] allLimelights = {frontLL}; // List of all limelights
    public final Limelight[] poseLimelights = {frontLL}; // Limelights specifically used for estimating pose
    public final Limelight[] reefLimelights = {frontLL}; // Effectively used for pose estimating, 
                                                        // but are specifically for use with the reef

    public boolean frontLLEnabled = frontLL.isAttached();
    public boolean backLLEnabled = backLL.isAttached();


    private final DecimalFormat df = new DecimalFormat();

    public boolean isIntegrating = false; // Boolean statign whether or not limelight poses are being integrated with actual

    public boolean isDriving = false; // Provides a boolean as to whether or not the Limelights are "driving" the robot with their measurements
    public String reefDriveTarget;

    // Creates an ArrayList to store estimated vision poses during autonomous 
    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();


    public SK25Vision(Optional<SKSwerve> m_swerveContainer) {
        this.m_swerve = m_swerveContainer.get();
        df.setMaximumFractionDigits(2);

        /* Limelight startup configurator*/
        for(Limelight ll : allLimelights) {
            if(!ll.isAttached()) {
                ll.setLogStatus("Disabled");
            }
            else {
                //ll.setLogStatus("Idle");
            }
            ll.setIMUMode(IMUMode.EXTERNAL); // For Limelight 4s only; this makes sure that we're only using the external IMU for Megatag 2
            ll.setLEDMode(false); // Turns off LED lights on startup
        }

        reefDriveTarget = "OFF";
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        SmartDashboard.putData("Vision",
        new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Limelights");

                builder.addStringProperty("BackLLStatus", () -> backLL.getLogStatus(), null);
                builder.addStringProperty("FrontLLStatus", () -> frontLL.getLogStatus(), null);
                builder.addStringProperty("ResetPoseToVisionStatus", () -> resetPoseToVisionLog, null);
            }
        });
    }

    // Unused
    public static final class AlignTranslationWithPose extends MultiLimelightCommandConfig {
        private AlignTranslationWithPose() {
            configKp(0.2);
            configKi(0.0);
            configKd(0.0);
            configTolerance(0.01);
            configMaxOutput(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.6);
            configError(0.3);
            configPipelineIndex(kAprilTagPipeline);
            configLimelights(RobotContainer.m_vision.poseLimelights);
        }

        public static AlignTranslationWithPose getConfig() {
            return new AlignTranslationWithPose();
        }
    }

    // Unused
    public static final class AlignToReefTag extends CommandConfig {
        private AlignToReefTag() {
            configKp(0.02);
            configKi(0.0);
            configKd(0.0);
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

    // public static final class DriveToReef extends MultiLimelightCommandConfig {
    //     private DriveToReef() {
    //         configKpid(0.2, 0, 0);
    //         configTolerance(0.01);
    //         configMaxOutput(TunerConstants.MaxSpeed * 0.75);
    //         configError(0.3);
    //         configPipelineIndex(kAprilTagPipeline);
    //         configLimelights(RobotContainer.m_vision.poseLimelights);
    //     }

    //     public static DriveToReef getConfig() {
    //         return new DriveToReef();
    //     }
    // }

    // public static final class RotateToReef extends MultiLimelightCommandConfig {
    //     private RotateToReef() {
    //         configKpid(0.02, 0, 0);
    //         configTolerance(0.01);
    //         configMaxOutput(TunerConstants.MaxAngularRate * 0.85);
    //         configError(0.3);
    //         configPipelineIndex(kAprilTagPipeline);
    //         configLimelights(RobotContainer.m_vision.poseLimelights);
    //     }

    //     public static RotateToReef getConfig() {
    //         return new RotateToReef();
    //     }
    // }

    public static final class DriveToPose extends MultiLimelightCommandConfig {
        private DriveToPose() {
            configKpid(6, 0, 0);
            configTolerance(0.05);
            configProfile(TunerConstants.MaxSpeed * 0.40, TunerConstants.MaxSpeed * 0.60); //60% Max Speed; 1.75x Acceleration
            configMaxOutput(TunerConstants.MaxSpeed * 0.40);
            configError(0.05);
            configPipelineIndex(kAprilTagPipeline);
            configLimelights(RobotContainer.m_vision.poseLimelights);
        }

        public static DriveToPose getConfig() {
            return new DriveToPose();
        }
    }

    public static final class RotateToPose extends MultiLimelightCommandConfig {
        private RotateToPose() {
            configKpid(6, 0.4, 0);
            configTolerance(0.01);
            configProfile((100) * TunerConstants.MaxAngularRate * 0.9, TunerConstants.MaxAngularRate * 1.5); // 90% Angular speed; 1.5x acceleration
            configMaxOutput((100)* TunerConstants.MaxAngularRate * 0.9);
            configError(0.01);
            configPipelineIndex(kAprilTagPipeline);
            configLimelights(RobotContainer.m_vision.poseLimelights);
        }

        public static RotateToPose getConfig() {
            return new RotateToPose();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("VisionDriving", isDriving);
        SmartDashboard.putString("ReefDriveTarget", reefDriveTarget);
        SmartDashboard.putString("BackLLStatus", backLL.getLogStatus());
        SmartDashboard.putString("FrontLLStatus", frontLL.getLogStatus());
        SmartDashboard.putString("ResetPoseToVisionStatus", resetPoseToVisionLog);

        for(Limelight ll : poseLimelights) {
            ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
        }

        /* The secret sauce: */
        estimatePose();
    }

    /**
     * Adds the estimated pose of a camera into an array to be stored for later usage.
     * <p>
     * Always use this within a for-loop of all cameras you wish to get the estimated pose with
     * and then pass the current camera into the method.
     * @param ll The camera to estimate the robot pose with
     */
    private void updatePoseMultiCam(Limelight ll) {
        // Sets the robot's yaw for use with MEGATAG2 right before integrating with estimator
        ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());

        Pose3d botPose3d = ll.getRawPose3d(); // Gets the 3D pose of the limelight on the robot using the position offsets
        Pose2d megaPose2d = ll.getMegaPose2d(); // Gets the estimated pose of the robot using MegaTag2 objects
        double timestamp = ll.getRawPoseTimestamp(); // Timestamp of when the pose was collected

        Pose2d estimatedRobotPose = // Uses limelight (effectively swerve) rotation value instead of estimated rotation value
            new Pose2d(megaPose2d.getTranslation(), megaPose2d.getRotation()); 
        
        autonPoses.add(Trio.of(botPose3d, estimatedRobotPose, timestamp)); // Adds both poses into the auto pose feeder

    }

    private void updatePoseAutonomous() {
        if(!DriverStation.isAutonomousEnabled()) {
            return;
        }
        for(Limelight ll : poseLimelights) {
            if(!ll.targetInView()) {
                continue;
            }
            // TODO: Change auto to multicam and create correct auto estimator when ready
            updatePoseSingleCam(ll);
        }
    }

    private void updatePoseSingleCam(Limelight ll) {
        try {
            isIntegrating = false;
            // Sets the robot's yaw for use with MEGATAG2 right before integrating with estimator
            ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
            addFilteredLimelightInput(ll);
            // A limelight's integrating status is determined by if it's received a valid status to integrate its pose
            isIntegrating |= ll.getConfig().isIntegrating();
        }
        catch (Exception e) {
            DriverStation.reportWarning("Vision: Attempted to access nonexistent vision pose!", false);
        }
    }

    private void updatePoseTeleop() {
        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        Limelight bestLL = getBestLimelight();
        for(Limelight ll : poseLimelights) {
            if (ll.getName() != bestLL.getName()) {
                ll.sendInvalidStatus("Rejected: Not best Limelight");
            }
        }

        updatePoseSingleCam(bestLL);    
    }

    public void estimatePose() {
        // Make sure robot orientation is correct
        /*
        Autonomous pose updater:
        
        For each limelight that has a tag in view, send it to be aggregated into the current pose estimate, alongside other
        recent poses.
        */
        //TODO: Reenable auto pose updater when ready
        // updatePoseAutonomous();

        /*
        Teleop pose updater:

        Instead of allowing all limelights to feed data into the SwervePoseEstimator, it scores each limelight that has a
        tag in view based on the tag's proximity to the bot and the number of tags seen. Only then does it feed a pose measurement 
        into the pose estimator. The pose can still be rejected for being too erroneous, but scoring each camera significantly
        reduces the chances of a pose estimate being rejected.
        */
        updatePoseTeleop();
    }

    public boolean reefTargetClose(Limelight ll) { 
        // RawFiducial[] tags = ll.getRawFiducial(); // Assuming we're on blue alliance
        int[] targetIDs = blueReefTagIDs;

        if(Field.isRed()) { // Check to make sure if Red Alliance
            targetIDs = redReefTagIDs;
        }

        double closestTagID = ll.getClosestTagID();

        /*Since all reef tags are sequential, just check upper
        and lower bound of reef tag sequence. If seen tag's id
        is part of the targeted reef tags, return true. */ 
        if(closestTagID <= targetIDs[5] && closestTagID >= targetIDs[0]) {
            return true;
        }

        // If none of the iterated tags on this limelight match our alliance's reef tags:
        return false;
    }

    public static enum FIELD_ELEMENT { // We don't really care how close the other tags are (processor, barge)
        REEF
    }

    /**
     * Finds the closest AprilTag for a specific field element for your alliance.
     * <p>
     * <b>SURROUND THIS IN TRY/CATCH OR IF/ELSE! It will return null if no AprilTags of specific target are seen!</b>
     * @param ll The limelight to reed tags from
     * @param element The field element's tag(s) to target
     * @return The closest matching tag found.
     */
    public RawFiducial getClosestTargetFiducial(Limelight ll, FIELD_ELEMENT element) {
        RawFiducial[] tags = {};
        try {
            tags = ll.getRawFiducial();
        }
        catch(NullPointerException e) {
            DriverStation.reportWarning("Limelight.getRawFiducial() returned null!", false);
        }
        ArrayList<RawFiducial> goodTags = new ArrayList<RawFiducial>();
        int[] targetIDs = {};

        if(tags.length == 0) {
            //TODO: make this report somewhere else
            DriverStation.reportWarning("Vision: " + ll.getName() + " Cannot find target tag; no tags seen!", false);
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
            DriverStation.reportWarning("Vision: [" + ll.getName() + "] No target tags found for " + element, false);
            return null;
        }
        else if(!goodTags.isEmpty()) {
            // TODO: Make this report somewhere else
            System.out.println("Vision: [" + ll.getName() + "] " + goodTags.size() + " target tag(s) found for " + element);
            if(goodTags.size() > 1) {
                System.out.print("Tag IDs: ");
                for(int i=0; i<goodTags.size(); i++) {
                    System.out.print(goodTags.get(i).id + " ");
                }
                System.out.println();
            }
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
        if(Robot.isSimulation()) {
            return;
        }
        Limelight ll = getBestLimelight();
        if(ll == null) {
            return;
        }
        if(!ll.targetInView()) {
            return;
        }

        ll.setRobotOrientation(ll.getRawPose3d().toPose2d().getRotation().getDegrees());
        //TODO: if MT2 doesn't work, change it to the line below
        // m_swerve.resetPose(ll.getRawPose3d().toPose2d());
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
                System.out.println("AutonResetPoseToVision succeeded on " + (autonPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            System.out.println(
                    "AutonResetPoseToVision failed after "
                            + batchSize
                            + " of "
                            + autonPoses.size()
                            + " possible tries");
            
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
        if(!ll.targetInView()) {
            return;
        }

        Pose3d botPose3d = ll.getRawPose3d();
        ll.setRobotOrientation(botPose3d.toPose2d().getRotation().getDegrees());
        resetPoseToVision(
                ll.targetInView(), botPose3d, ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    private String resetPoseToVisionLog = "Not executed yet..."; // Provides an updatable string for smartdashboard
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
            if (Field.poseOutOfField(megaPose)
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

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), megaPose.getRotation());
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
        if (!LL.targetInView()) {
            LL.setTagStatus("no tags");
            LL.sendInvalidStatus("no tag found rejection");
            return;
        }

        LL.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
        boolean multiTags = LL.multipleTagsInView();
        double timeStamp = LL.getRawPoseTimestamp();
        double targetSize = LL.getTargetSize();
        Pose3d botpose3DMT1 = LL.getRawPose3d();
        Pose2d botposeMT1 = botpose3DMT1.toPose2d();
        Pose2d botposeMT2 = LL.getMegaPose2d();
        RawFiducial[] tags = LL.getRawFiducial();
        double highestAmbiguity = 2;
        ChassisSpeeds robotSpeed = m_swerve.getRobotRelativeSpeeds();

        // distance from current pose to vision estimated pose
        double poseDifference =
                m_swerve.getRobotPose().getTranslation().getDistance(botposeMT2.getTranslation());

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
        if (Field.poseOutOfField(botposeMT2)) {
            // reject if pose is out of the field
            LL.sendInvalidStatus("bound rejection - pose OOB");
            return;
        } else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 4 * Math.PI) {
            // reject if we are rotating more than 0.5 rad/s
            LL.sendInvalidStatus("rot speed rejection - too fast");
            return;
        } else if (Math.abs(botpose3DMT1.getZ()) > 0.25) {
            // reject if pose is .25 meters in the air
            LL.sendInvalidStatus("height rejection - in air");
            return;
        } else if (Math.abs(botpose3DMT1.getRotation().getX()) > 5
                || Math.abs(botpose3DMT1.getRotation().getY()) > 5) {
            // reject if pose is 5 degrees titled in roll or pitch
            LL.sendInvalidStatus("roll/pitch rejection - tilted");
            return;
        } else if (targetSize <= 0.025) {
            LL.sendInvalidStatus("size rejection - target too small");
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

        Pose2d integratedPose = new Pose2d(botposeMT2.getTranslation(), botposeMT2.getRotation());

        addVisionMeasurementWithStdDevs(
            integratedPose, 
            timeStamp, 
            VecBuilder.fill(
                VisionConfig.VISION_STD_DEV_X,
                VisionConfig.VISION_STD_DEV_Y,
                VisionConfig.VISION_STD_DEV_THETA));
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
        addVisionMeasurementWithStdDevs(integratedPose, timeStamp, stdDevs);
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
        public double maxVelocity;
        public double maxAcceleration;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        /**
         * Configures the pid constants of the controller using PathPlanner Autonomous
         * PIDConstants.
         * @param Constants PIDConstants to set the controller's PID values to.
         */
        public void configKpid(PIDConstants Constants) {
            this.kp = Constants.kP; this.ki = Constants.kI; this.kd = Constants.kD;
        }

        /**
         * Configures the pid constants of the controller using three values
         * @param kp The P value of the controller
         * @param ki The I value of the controller
         * @param kd the D value of the controller
         */
        public void configKpid(double kp, double ki, double kd) {
            this.kp = kp; this.ki = ki; this.kd = kd;
        }

        /**
         * Configures only the P value of the PID controller
         * @param kp The P value of the controller
         */
        public void configKp(double kp) {
            this.kp = kp;
        }

        /**
         * Configures only the I value of the PID controller
         * @param ki The I value of the controller
         */
        public void configKi(double ki) {
            this.ki = ki;
        }

        /**
         * Configures only the D value of the PID controller
         * @param kd The D value of the controller
         */
        public void configKd(double kd) {
            this.kd = kd;
        }

        /**
         * Configures the Trapezoidal motion profile of the controller
         * @param maxVel The maximum velocity to configure the profile with
         * @param maxAccel The maximum acceleration to configure the profile with
         */
        public void configProfile(double maxVel, double maxAccel) {
            this.maxVelocity = maxVel;
            this.maxAcceleration = maxAccel;
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

    public static class MultiLimelightCommandConfig extends CommandConfig {
        public Limelight[] limelights;
        public void configLimelights(Limelight... limelights) {
            this.limelights = limelights;
        }
    }
}
