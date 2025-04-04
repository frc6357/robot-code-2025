package frc.robot.commands;

import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kElevatorHeightBottomLimit;
import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kElevatorHeightTopLimit;
import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kManualElevatorDeadband;
import static frc.robot.Konstants.ElevatorConstants.CoralSubsystemConstants.CoralSubsystem.kManualElevatorSpeedScalar;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

// import com.revrobotics.RelativeEncoder;

public class CoralElevatorJoystickCommand extends Command {
    private final CoralSubsystem elevator;
    private final Supplier<Double> joystickInput;
    private double posDelta;
    private double newTargetHeight;


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
        if (Math.abs(joystickInput.get()) > kManualElevatorDeadband)
        {
            posDelta = joystickInput.get() * kManualElevatorSpeedScalar; // Units to adjust to periodic loop: 20ms x 50 = 1s

            newTargetHeight = elevator.elevatorEncoder.getPosition() + posDelta;

            // DriverStation.reportError("NEW HEIGHT: " + String.valueOf(joystickInput.get()), false);

            if (newTargetHeight >= kElevatorHeightBottomLimit && newTargetHeight <= kElevatorHeightTopLimit + 1.0) //1 motor rotation tollerance
                elevator.setTargetHeight(newTargetHeight);  //elevator in bounds
            else if (newTargetHeight < kElevatorHeightBottomLimit)
                elevator.setTargetHeight(kElevatorHeightBottomLimit);  //min elevator height
            else 
                elevator.setTargetHeight(kElevatorHeightTopLimit);   //max elevator height
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}

