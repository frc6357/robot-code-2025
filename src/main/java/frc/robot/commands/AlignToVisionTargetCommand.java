package frc.robot.commands;

import static frc.robot.Konstants.VisionConstants.kAprilTagPipeline;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.SK25Vision.CommandConfig;
import frc.robot.utils.vision.Limelight;
import frc.robot.commands.DriveCommand;

public class AlignToVisionTargetCommand extends Command {
    private CommandConfig config;
    private Limelight limelight;
    DriveCommand driveCommand;
    Supplier<Double> fwdPositiveSupplier; // This is a robot centric foward/backwards
    private double out = 0;
    private double heading = Integer.MIN_VALUE; // Heading of the robot's rotation when driving; often unused
    private double horizontalSetpoint; // Goal tx of vision target
    private boolean outputting;

    private SlewRateLimiter slewFilter;

    private PIDController xPID; // PID Controller for the robot to move on its x-axis (Robot centric driving)

    // Because driving with vision is robot centric, aligning involves specifically only moving on the robot's x-plane to align with a target

    public AlignToVisionTargetCommand(CommandConfig config, Supplier<Double> fwdPositiveSupplier, double horizontalSetpoint) {
        this.config = config;
        this.limelight = config.limelight;

        xPID = new PIDController(config.kp, config.ki, config.kd);
        xPID.setTolerance(config.tolerance);

        this.fwdPositiveSupplier = fwdPositiveSupplier;

        this.horizontalSetpoint = horizontalSetpoint; // Sets the horizontal goal offset (tx) of the target to the limelight (in degrees)

        slewFilter = new SlewRateLimiter(1.5);

        addRequirements();

        driveCommand =
                new DriveCommand(
                    () -> getOutput(), // PID controller left/right motion
                    fwdPositiveSupplier, // Driver-requested forward/backward motion 
                    () -> (0.0), // No rotational motion
                    () -> (false)); // Robot oriented
    }

    @Override
    public void initialize() {
        xPID.reset();
        out = 0;

        limelight.setLimelightPipeline(config.pipelineIndex);
    }

    @Override
    public void execute() {
        if(!atTarget() && outputting) {
            setOutput(xPID.calculate(limelight.getHorizontalOffset(), horizontalSetpoint));
        }
        else {
            setOutput(0);
        }
        
        driveCommand.run();
    }


    public void setOutput(double output) {
        this.out = output;
        if(Math.abs(out) > 1) {
            out = 1 * Math.signum(out);
        }

        out = slewFilter.calculate(out); // Applies the slew filter to the output magnitude
        
        // Multiply the output magnitude by the allowed speed
        this.out *= config.maxOutput; // maxOutput is the maximum allowed drivetrain speed for the specific command
    }

    public double getOutput() {
        return out;
    }

    public boolean atTarget() {
        if(xPID.atSetpoint() == true) {
            outputting = false;
        }
        return xPID.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {
        outputting = false;
        limelight.setLimelightPipeline(kAprilTagPipeline);
    }
}
