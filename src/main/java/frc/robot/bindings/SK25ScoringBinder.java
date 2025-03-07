package frc.robot.bindings;

import frc.robot.commands.commandGroups.L4ScoreCommandGroup;
import frc.robot.commands.commandGroups.ScoreCommandGroup;

import static frc.robot.Konstants.ElevatorConstants.ElevatorPosition.*;
import static frc.robot.Konstants.EndEffectorConstants.EndEffectorPosition.*;
import static frc.robot.Ports.OperatorPorts.*;

import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SK25Elevator;

public class SK25ScoringBinder implements CommandBinder {

    Optional<SK25EndEffector> endEffectorSubsystem;
    Optional<SK25Elevator> elevatorSubsystem;

    Trigger lowPositionButton;
    Trigger topPositionButton;
    Trigger midPositionButton;
    Trigger troughPositionButton;
    Trigger intakePositionButton;
    Trigger zeroPositionButton;
    Trigger netPositionButton;

    RelativeEncoder mEncoder;

    public SK25ScoringBinder(Optional<SK25EndEffector> endEffectorSubsystem, Optional<SK25Elevator> elevatorSubsystem)
    {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        this.zeroPositionButton = kZeroPositionOperator.button;
        this.lowPositionButton = kLowBranch.button;
        //this.midPositionButton = kMiddleBranch.button;
        this.intakePositionButton = kIntakePos.button;
        //this.topPositionButton = kTopBranch.button;
        this.troughPositionButton = kTrough.button;
        this.netPositionButton = kNetPos.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent() && elevatorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();
            SK25Elevator elevator = elevatorSubsystem.get();

            // endEffector Position Buttons
            /*netPositionButton.onTrue(new ScoreCommandGroup(kNetPosition, elevator, kTopPositionAngle, endEffector));
            topPositionButton.onTrue(new L4ScoreCommandGroup(kTopPosition, elevator, kTopPositionAngle, endEffector));    
            midPositionButton.onTrue(new ScoreCommandGroup(kMidPosition, elevator, kMidLowPositionAngle, endEffector));
            intakePositionButton.onTrue(new ScoreCommandGroup(kIntakePosition, elevator, kIntakePositionAngle, endEffector));
            lowPositionButton.onTrue(new ScoreCommandGroup(kLowPosition, elevator, kMidLowPositionAngle, endEffector));
            troughPositionButton.onTrue(new ScoreCommandGroup(kTroughPosition, elevator, kTroughPositionAngle, endEffector));
            zeroPositionButton.onTrue(new ScoreCommandGroup(kZeroPosition, elevator, kZeroPositionAngle, endEffector));          
            */
            }
    }
}