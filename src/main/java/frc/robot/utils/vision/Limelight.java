package frc.robot.utils.vision;

// A Limelight class provided by FRC 3847 Spectrum that aids with
// several Limelight configurations, measurements, and get/set methods

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.vision.LimelightHelpers.LimelightResults;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;

public class Limelight {

    /* Limelight Configuration */

    //@Getter and @Setter annotations create and call both get and set methods on the object annotated.
    //These annotations are rpovided by project lombok, documentation can be found on theri site.

    public static class LimelightConfig {
        /** The name of the limelight, must match to the name given in LL dashboard */
        @Getter @Setter private String name;
        /** If the limelight is attached to the robot or not*/
        @Getter @Setter private boolean attached = true;
        /** isIntegrating */
        @Getter @Setter private boolean isIntegrating;
        /** Physical Config : The distances of the limelight from the center of the robot.
         * Uses foward, right, up in meters where the specified directions are positive */
        @Getter private double forward, right, up; // meters
        /** The angle of the limelight in terms of roll, pitch, and yaw respectivley in degrees*/
        @Getter private double roll, pitch, yaw; // degrees

        /** Creates a new limelight config (configurable limelight)
         * @param name The name of the limelight
         */
        public LimelightConfig(String name) {
            this.name = name;
        }

        /**
         * Applies the translation of the limelight from the center of the robot.
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return The object this method is called on after withTranslation has been applied
         */
        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * Applies the rotation of the limelight from its default position.
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return The object this method is called on after withRotation has been applied
         */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }

        /**
         * Applies the attached or unattatched state of the limelight.
         * @param attached whether or not the limelight is attached
         * @return The object this method is called on after withAttached has been applied
         */
        public LimelightConfig withAttached(boolean attached) {
            this.attached = attached;
            return this;
        }
    }

    /* Debug */
    private final DecimalFormat df = new DecimalFormat();
    @Getter private LimelightConfig config;
    @Getter @Setter private String logStatus = "";
    @Getter @Setter private String tagStatus = "";

    /** Creates a new limelight object.
     * @param config The limeight config object to use
     */
    public Limelight(LimelightConfig config) {
        this.config = config;
    }

    /** Creates a new limelight object.
     * @param name The name of the limelight / limeight config to use
     */                                         
    public Limelight(String name) {
        config = new LimelightConfig(name);
    }

    /** Creates a new limelight object.
     * @param name The name of the limelight / limeight config to use
     * @param attached The state of the limelight (attached or not)
     */
    public Limelight(String name, boolean attached) {
        config = new LimelightConfig(name).withAttached(attached);
    }

    /** Creates a new limelight object.
     * @param cameraName The name of the limelight / limeight config to use
     * @param pipeline The default pipeline to assign the limelight to
     */
    public Limelight(String cameraName, int pipeline) {
        this(cameraName);
        setLimelightPipeline(pipeline);
    }

    /**Gets the name of the limelight
     * @return the name of the limelight
     */
    public String getName() {
        return config.getName();
    }

    /**Gets the attached state of the limelight
     * @return the attached state of the limelight
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /* ::: Basic Information Retrieval ::: */
    /**Gets the horizontal offset of the crosshair from the target using getTX() from the limelight helpers class.
     * If the limelight is not attached, a value of zero is returned.
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2:
     *     -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTX(config.getName());
    }

    /**
     * Gets the vertical offset of the crosshair from the target using getTY() from the limelight helpers class.
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees). 
     * If the limelight is not attached, a value of zero is returned.
     */
    public double getVerticalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTY(config.getName());
    }

    /** 
     * Determines if any valid targets are in veiw of the limelight (specified by pipelines) using getTV() from the limelight helpers class.
     * @return Whether the LL has any valid targets (April tags or other vision targets) 
     * If the limelight is not attached, return false.*/
    public boolean targetInView() {
        if (!isAttached()) {
            return false;
        }
        return LimelightHelpers.getTV(config.getName());
    }

    /** 
     * Checks if multiple targets are veiwable by the limelight.
     * @return whether the LL sees multiple tags or not.
     * If the limelight is not attached, return false.*/
    public boolean multipleTagsInView() {
        if (!isAttached()) {
            return false;
        }
        return getTagCountInView() > 1;
    }

    /** 
     * Gets the amount of targets veiwable by the limelight using a robot pose estimate with getBotPoseEstimate() from LimelightHelpers.
     * @return whether the LL sees multiple tags or not.
     * If the limelight is not attached, return false.*/
    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName()).tagCount;

        // if (retrieveJSON() == null) return 0;

        // return retrieveJSON().targetingResults.targets_Fiducials.length;
    }

    /**
     * Gets the limelight tag at the centermost point of its veiw using getFiducialID() from the limelighthelpers class.
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     * If the limelight is not attahced, return zero.
     */
    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    /**
     * Gets the target tag area using getTA() from the limelighthelpers class. The target tag area is the 
     * percentage of the window visible by the camera taken up by the tag, where 100% is the full window
     * and 0% means it cannot see a tag.
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     * If the limelight is not attahced, return zero.
     */
    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTA(config.getName());
    }

    /* ::: Pose Retrieval ::: */

    /** Gets the limelight pose by using the current robot pose and accounting for the limelight's
     * offset from the center of the robot. This uses getBotPose3d_wpiBlue() from the limelight helpers class.
     * @return the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java 
     * If no limelight is attached, return a Pose3d() object with no translatoin or rotation values.*/
    public Pose3d getRawPose3d() {
        if (!isAttached()) {
            return new Pose3d();
        }
        return LimelightHelpers.getBotPose3d_wpiBlue(
                config.name); // 2024: all alliances use blue as 0,0
    }

    /**
     * Gets the april tag pose2d by using the current robot pose and accounting for the limelight's
     * offset from the center of the robot. This uses getBotPose3d_wpiBlue() from the limelight helpers class,
     * including logic with the MegaTag2 object which runs these calculations given the robot pose. 
     * @return the corresponding LL Pose2d (MEGATAG2) for the alliance in DriverStation.java 
     * If no limelight is attached, return a Pose2d() object with no translatoin or rotation values.*/
    public Pose2d getMegaPose2d() {
        if (!isAttached()) {
            return new Pose2d();
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name).pose; // 2024: all alliances use blue as 0,0
        
        //TODO: see if starting blue alliance corrds have changed
    }

    /** Leverages the limelight's veiw of multiple tags and their distance from the robot to check if the
     * robot pose and/or limelight pose are more accurate than a basic pose update from the gyro/accelerometer.
     * @retrun If the position is "accurate".
     * If no limelight is attached, return false.
     */
    public boolean hasAccuratePose() {
        if (!isAttached()) {
            return false;
        }
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /** Determines the distance from the tag and the limelight as a vector. We can use x and y distances
     * and pythagorean therom to determines the distance in a straight line.
     * @return the distance of the 2d vector from the camera to closest apriltag
     *  If no limelight is attached, return a distance of zero. */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /** Gets an array of the raw network table ouput (the raw april tag data)*/           //TODO: need more understanding
    public RawFiducial[] getRawFiducial() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name).rawFiducials;
    }

    /**
     * Returns the timestamp of the MEGATAG1 pose estimation from the Limelight camera.
     * @return The timestamp of the pose estimation in seconds.
     *  If no limelight is attached, return a time value of zero.
     */
    public double getRawPoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName()).timestampSeconds;
    }

    /**
     * Returns the timestamp of the MEGATAG2 pose estimation from the Limelight camera.
     * @return The timestamp of the pose estimation in seconds.
     *  If no limelight is attached, return a distance of zero.
     */
    public double getMegaPoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName())
                .timestampSeconds;
    }

    /**
     * Returns the latency of the pose estimation from the Limelight camera.
     * @return The latency of the pose estimation in seconds.
     *  If no limelight is attached, return a distance of zero.
     */
    @Deprecated(forRemoval = true)  //declare this method as deprecated for removal
    public double getPoseLatency() {
        if (!isAttached()) {
            return 0;
        }
        return Units.millisecondsToSeconds(
                LimelightHelpers.getBotPose_wpiBlue(config.getName())[6]);
    }

    /*
     * Custom Helpers
     */

    /**
     * Get distance in meters to a target
     * @param targetHeight height of the target in meters
     * @return the distance of the liemlight to the target in meters.
     *  If no limelight is attached, return a distance of zero.
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.roll + getVerticalOffset()));
    }

    /** Log the validity of the limelight as valid. Used when any and all
     *  required conditions for the limelight are met.
     * @param message The message to display alongside the status signal.  */
    public void sendValidStatus(String message) {
        config.isIntegrating = true;
        logStatus = message;
    }

    /** Log the validity of the limelight as invalid. Used when any
     *  required conditions for the limelight are not met.
     * @param message The message to display alongside the status signal.*/
    public void sendInvalidStatus(String message) {
        config.isIntegrating = false;
        logStatus = message;
    }

    /*
     * Utility Wrappers
     */

    /** @return The latest LL results as a LimelightResults object. */
    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    /** Sets the limelight target pipeline. Nothing happens if the limelight is not attached.
     * @param pipelineIndex use pipeline indexes in {@link VisionConfig} */
    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    //TODO: change Limelight calls for setting the swerve drivetrain

    public void setRobotOrientation(double degrees) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * Sets the LED mode of the LL.
     *  If no limelight is attached, nothing will happen.
     * @param enabled true to enable the LED mode, false to disable it
     */
    public void setLEDMode(boolean enabled) {
        if (!isAttached()) {
            return;
        }
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(config.getName());
        } else {
            LimelightHelpers.setLEDMode_ForceOff(config.getName());
        }
    }

    /**
     * Set LL LED's to blink
     */
    public void blinkLEDs() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setLEDMode_ForceBlink(config.getName());
    }

    /** Checks if the camera is connected by looking for an empty botpose array from camera. 
     * @return if the camera is connected
    */
    public boolean isCameraConnected() {
        if (!isAttached()) {
            return false;
        }
        try {
            var rawPoseArray =
                    LimelightHelpers.getLimelightNTTableEntry(config.getName(), "botpose_wpiblue")
                            .getDoubleArray(new double[0]);
            if (rawPoseArray.length < 6) {
                return false;
            }
            return true;
        } catch (Exception e) {
            System.err.println("Avoided crashing statement in Limelight.java: isCameraConnected()");
            return false;
        }
    }

    /** Prints the vision, estimated, and odometry pose to SmartDashboard */
    public void printDebug() {
        if (!isAttached()) {
            return;
        }
        Pose3d botPose3d = getRawPose3d();
        SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
        SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
        SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
        SmartDashboard.putString(
                "LimelightRoll", df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
        SmartDashboard.putString(
                "LimelightPitch",
                df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
        SmartDashboard.putString(
                "LimelightYaw", df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
    }
}