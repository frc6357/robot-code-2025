package frc.robot.commands;

import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kMaxElevatorSpeed;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.CoralSubsystem;

// import com.revrobotics.RelativeEncoder;

public class CoralElevatorJoystickCommand extends Command {
    private final CoralSubsystem elevator;
    private final Supplier<Double> joystickInput;
    private double speed;


    public CoralElevatorJoystickCommand(Supplier<Double> axis, CoralSubsystem elevator)
    {
        this.joystickInput = axis;
        this.elevator = elevator;

        addRequirements(elevator);

        

        
    }

    @Override 
    public void initialize(){}

    @Override
    public void execute()
    {
        // //increase the elevator height based on joystick tilt.
        // speed = kMaxElevatorSpeed * joystickInput.get();
        // elevator.setManualSetpointCommand(speed);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}

