package frc.robot.commands;

import static frc.robot.Ports.DriverPorts.kDriver;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.utils.vision.Limelight;

public class RotateToTag extends Command{
    private ProfiledPIDController rotPID;
    private SKSwerve m_swerve;
    private MultiLimelightCommandConfig rotConfig;
    private Constraints rotConstraints;
    private DriveCommand driveCommand;
    private SK25Vision m_vision;
    private Limelight[] limelights;
    private Limelight targetLimelight;
    private boolean valid;

    private boolean killed = false;

    private double rotOut;

    public RotateToTag(
        MultiLimelightCommandConfig rotConfig,
        SK25Vision m_vision,
        SKSwerve m_swerve
    ) 
    {
        this.rotConfig = rotConfig;
        this.m_vision = m_vision;
        this.m_swerve = m_swerve;
        this.limelights = rotConfig.limelights;

        rotConstraints = new Constraints(rotConfig.maxVelocity, rotConfig.maxAcceleration);

        rotPID = new ProfiledPIDController(
            rotConfig.kp,
            rotConfig.ki,
            rotConfig.kd,
            rotConstraints);

        rotPID.setTolerance(rotConfig.tolerance);

        rotPID.enableContinuousInput(-180, 180);

        this.driveCommand = new DriveCommand(
            () -> 0.0, 
            () -> 0.0, 
            () -> getRotOutput(), 
            () -> false);
        
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        setLimelight();
        reset();

        if(!valid) {
            killed = true;
        }
    }

    @Override
    public void execute() {
        // Check for the limelight's validity before moving on
        if(!killed) {
            valid = targetLimelight.targetInView();
        }
        // If current targetLimelight is no longer valid (can't see target),
        // redetermine the best limelight
        if(!valid && !killed) {
            setLimelight();
        }
        // After checking all available limelights for a tag, only run this 
        // body if we have a good limelight after the check
        if(valid) {
            kDriver.setRumble(RumbleType.kBothRumble, 0.5);
            calculateOutput();

            m_vision.isDriving = true;

            driveCommand.run();
        }
        else {
            // Kills the command because it has already tried to find 
            // another good limelight by this point and failed to do so
            killed = true;
        }
        SmartDashboard.putNumber("Align/rotPIDOutput", rotOut);
    }

    @Override
    public void end(boolean isInterrupted) {
        kDriver.setRumble(RumbleType.kBothRumble, 0.0);
        m_vision.isDriving = false;

        if(valid) {
            shutdown();
        }
    }

    @Override
    public boolean isFinished() {
        return killed;
    }

    private double getRotOutput() {
        return rotOut;
    }

    private void calculateOutput() {
        double[] positions = targetLimelight.getRobotPoseTS();
        SmartDashboard.putNumber("Align/RotDeg", positions[4]);

        rotOut = rotConfig.maxVelocity * -rotPID.calculate(positions[4]);
        if(Math.abs(positions[4] - rotPID.getGoal().position) <= rotConfig.tolerance) {
            rotOut = 0;
        }
    }

    private void setLimelight() {
        findGoodLimelight().ifPresentOrElse(
            (ll) -> {
                targetLimelight = ll;
                valid = true;
            },
            () -> {
                targetLimelight = null;
                valid = false;
            });
    }

    private void reset() {
        if(valid) {
            rotPID.setGoal(0.0);
            rotPID.reset(targetLimelight.getRobotPoseTS()[4], m_swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond * (180/Math.PI));
        }
    }

    private void shutdown() {
        rotPID.setGoal(targetLimelight.getRobotPoseTS()[4]);
    }

    private Optional<Limelight> findGoodLimelight() {
        Limelight goodLimelight = null;
        for(Limelight ll : limelights) {
            if(ll.targetInView()) {
                goodLimelight = ll;
            }
        }
        return Optional.ofNullable(goodLimelight);
    }
}
