package frc.robot.commands.VisionCommands.DriveToPose;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.Field;
import frc.robot.utils.vision.Limelight;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import static frc.robot.Ports.DriverPorts.kLeftReef;
import static frc.robot.Ports.DriverPorts.kRightReef;
import static frc.robot.Ports.DriverPorts.kDriver;
import static frc.robot.Ports.OperatorPorts.kOperator;
import static frc.robot.Ports.DriverPorts.kSlowMode;
import static frc.robot.Ports.DriverPorts.kDriveFn;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;
import static frc.robot.Ports.DriverPorts.kVelocityOmegaPort;


import frc.robot.Konstants.TunerConstants;
import frc.robot.Konstants.VisionConstants.PoseConstants;

public class DriveToReefPoseCommand extends Command{
    MultiLimelightCommandConfig driveConfig;
    MultiLimelightCommandConfig rotateConfig;

    Limelight[] limelights;

    RotateToPose rotateController;
    TranslateToPose driveController;

    Pose2d targetPose;

    SK25Vision m_vision;
    SKSwerve m_swerve;

    DriveCommand driveCommand;

    boolean valid;

    Trigger targetLeftSide = kLeftReef.button;
    Trigger targetRightSide = kRightReef.button;

    Trigger slowMode = kSlowMode.button.and(kDriveFn.button);
    boolean slowModeStatus = false;

    RawFiducial closestTag;

    static enum Target {
        CENTER,
        LEFT,
        RIGHT
    }

    Target prevTarget;
    Target currentTarget;

    private void setCurrentTarget() {
        if(targetLeftSide.and(targetRightSide).getAsBoolean() || // Both triggers pressed
        targetLeftSide.negate().and(targetRightSide.negate()).getAsBoolean()) { // Neither trigger pressed
            currentTarget = Target.CENTER;
        }
        else if(targetLeftSide.getAsBoolean()) {
            currentTarget = Target.LEFT;
        }
        else {
            currentTarget = Target.RIGHT;
        }
    }

    /* Methods for getting the driver's rotational input */
    public double applyGains(double axis, double slowPercent)
    {
        slowModeStatus = slowMode.getAsBoolean();
        if (slowModeStatus)
        {
            return axis * slowPercent;
        }
        else
            return axis;
    }

    /**
     * 
     * @param driveConfig The specific profiled PID and limelight config to use for translating the robot
     * @param rotateConfig The specific profiled PID and limelight config to use for rotating the robot
     * @param m_vision The vision instance used to run tag recognition and distance calculations
     * @param m_swerve The swerve instance to drive
     * 
     * @return A command to override the swerve's default command and use PID loops to move the robot
     * chassis to a known position on the reef based on the tags the pose limelights see.
     */
    public DriveToReefPoseCommand(
                MultiLimelightCommandConfig driveConfig,
                MultiLimelightCommandConfig rotateConfig, 
                SK25Vision m_vision, 
                SKSwerve m_swerve) {

        this.driveConfig = driveConfig;
        this.rotateConfig = rotateConfig;
        this.m_vision = m_vision;
        this.m_swerve = m_swerve;

        
        setCurrentTarget(); // Call this at the beginning of every loop as well
        prevTarget = currentTarget; // Call this at the end of every loop as well

        // Since we will be driving and rotating at the same time, the drive type will need to be field-centric
        this.driveCommand = new DriveCommand(
            () -> (driveController.getXOutput()), // driveController.getXOutput()
            () -> (driveController.getYOutput()), // driveController.getYOutput()
            () -> (rotateController.getOutput()), // applyGains(TunerConstants.MaxAngularRate * -1.0 * kVelocityOmegaPort.getFilteredAxis(), kSlowModePercent)
            () -> (true));
        
        // Drive config and rotate config both use the same limelights, so only need to call one config's array here
        this.limelights = driveConfig.limelights;

        addRequirements(m_swerve);
    }

    /* Switches the target of the reef pose while maintaining target reef face */
    private void setTargetPose() {
        List<String> targetPositions = PoseConstants.tagDestinationMap.get(closestTag.id);
        String targetPosition = "";

        /* Effectively prioritizes left side scoring */
        if(currentTarget == Target.LEFT) {
            m_vision.reefDriveTarget = "LEFT";

            targetPosition = targetPositions.get(0); // Left position is at index 0
        }
        else if(currentTarget == Target.RIGHT) {
            m_vision.reefDriveTarget = "RIGHT";

            targetPosition = targetPositions.get(1); // Right position is at index 1
        }
        else {
            m_vision.reefDriveTarget = "CENTER";

            targetPosition = targetPositions.get(2); // Center position is at index 2
        }

        targetPose = PoseConstants.fieldPositions.get(targetPosition);


        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) { // Should we flip pose? 
            targetPose = new Pose2d(
                    Field.flipXifRed(targetPose.getX()),
                    Field.flipYifRed(targetPose.getY()),
                    targetPose.getRotation().plus(Rotation2d.kPi));
        }
        
        
        SmartDashboard.putNumber("LastTargetPoseX", targetPose.getX());
        SmartDashboard.putNumber("LastTargetPoseY", targetPose.getY());


        driveController.initialize(targetPose);
        rotateController.initialize(targetPose);
    }

    ArrayList<RawFiducial> reefTags;
    @Override
    public void initialize() {
        reefTags = new ArrayList<RawFiducial>();
        closestTag = null;

        for(Limelight ll : limelights) {
            RawFiducial closestReefTag = m_vision.getClosestTargetFiducial(ll, SK25Vision.FIELD_ELEMENT.REEF);
            // getClosestTargetFiducial() will return null if no targets seen
            if(closestReefTag != null) {
                reefTags.add(closestReefTag);
            }
        }

        // Assuming that there is a valid reef tag in sight:
        if(!reefTags.isEmpty()) {
            closestTag = reefTags.get(0);
            double closestTagDistance = closestTag.distToRobot;

            if(reefTags.size() > 1) { // No need to for loop if only one tag seen
                for(RawFiducial reefTag : reefTags) {
                    if(reefTag.distToRobot < closestTagDistance) {
                        closestTag = reefTag;
                        closestTagDistance = closestTag.distToRobot;
                    }
                }
            }

            System.out.println("Closest Reef Tag: " + closestTag.id);
            valid = true;
        }
        else {
            valid = false;
            DriverStation.reportWarning("DriveToReefCommand failed. No reef tags seen", false);
        }


        this.rotateController = new RotateToPose(rotateConfig, m_swerve);
        this.driveController = new TranslateToPose(driveConfig, m_swerve);

        if(valid) {
            m_vision.resetPoseToVision();
            setTargetPose();
        }
    }

    @Override
    public void execute() {
        if(valid) {
            setCurrentTarget();

            m_vision.isDriving = true;
            // We don't want the controller rumbling during auto
            if(DriverStation.isTeleopEnabled()) {
                kDriver.setRumble(RumbleType.kBothRumble, 0.5);

                if(driveController.isFinished())  {
                    // This signals to the operator that vision is done aligning and is ready to score
                    kOperator.setRumble(RumbleType.kBothRumble, 0.5);
                }
            }

            // If the driver has pressed a different button
            if(currentTarget != prevTarget) {
                setTargetPose();
            }

            driveCommand.run();

            prevTarget = currentTarget;

            SmartDashboard.putNumber("PIDx", driveController.getXGoal());
            SmartDashboard.putNumber("PIDy", driveController.getYGoal());
            SmartDashboard.putNumber("PIDtheta", rotateController.getGoal());
        }
        else {
            kOperator.setRumble(RumbleType.kBothRumble, 0.0);
            kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if(!valid) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_vision.isDriving = false;
        m_vision.reefDriveTarget = "OFF";
        kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        kOperator.setRumble(RumbleType.kBothRumble, 0.0);
        driveController.end();  
        rotateController.end();
    }

}
