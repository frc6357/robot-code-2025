package frc.robot.subsystems.vision;

// Used for standard deviation matrix
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// import static frc.robot.Konstants.VisionConstants.limelightAlpha;
// import static frc.robot.Konstants.VisionConstants.limelightBeta;
import static frc.robot.Konstants.VisionConstants.*;
import frc.robot.utils.vision.Limelight.LimelightConfig;

public final class VisionConfig {
    public static final String BACK_LL = limelightAlpha.kName;
    public static final int BACK_TAG_PIPELINE = limelightAlpha.kAprilTagPipeline;
    public static final LimelightConfig BACK_CONFIG = 
                                        new LimelightConfig(limelightAlpha.kName) // Yes, it's the same as [DIRECTION]_LL. Just left it like this to see constructor layout
                                        .withTranslation(limelightAlpha.kForward, limelightAlpha.kRight, limelightAlpha.kUp) // Feeds in the position of the limelight on the bot
                                        .withRotation(limelightAlpha.kRoll, limelightAlpha.kPitch, limelightAlpha.kYaw) // Feeds in rotation of limelight
                                        .withAttached(limelightAlpha.kAttached); // Whether or not the limelight is attached to the robot; if false, effectively disables limelight


    public static final String FRONT_LL = limelightBeta.kName;
    public static final int FRONT_TAG_PIPELINE = limelightBeta.kAprilTagPipeline;
    public static final LimelightConfig FRONT_CONFIG = 
                                        new LimelightConfig(limelightBeta.kName)
                                        .withTranslation(limelightBeta.kForward, limelightBeta.kRight, limelightBeta.kUp)
                                        .withRotation(limelightBeta.kRoll, limelightBeta.kPitch, limelightBeta.kYaw)
                                        .withAttached(limelightBeta.kAttached);

    public static double VISION_REJECT_DISTANCE = kVisionRejectDist;

    public static double VISION_STD_DEV_X = 0.5; // These are not final because they are sometimes (not often) changed 
    public static double VISION_STD_DEV_Y = 0.5; // on the fly for pose estimating from vision measurements.
    public static double VISION_STD_DEV_THETA = 99999999; // The smaller the number, the stricter the measurement requirements.

    public static final Matrix<N3, N1> visionStdMatrix = // This however, creates a matrix using these standard values to reference for that other 90% of the time
            VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
}
