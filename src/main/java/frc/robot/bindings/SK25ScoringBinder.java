package frc.robot.bindings;

import frc.robot.commands.EndEffectorButtonCommand;
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

    RelativeEncoder mEncoder;

    public SK25ScoringBinder(Optional<SK25EndEffector> endEffectorSubsystem, Optional<SK25Elevator> elevatorSubsystem)
    {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        this.zeroPositionButton = kZeroPositionOperator.button;
        this.lowPositionButton = kLowBranch.button;
        this.midPositionButton = kMiddleBranch.button;
        this.intakePositionButton = intakebut.button;
        this.topPositionButton = kTopBranch.button;
        this.troughPositionButton = kTrough.button;
    }

    public void bindButtons()
    {
        // If subsystem is present then this method will bind the buttons
        if (endEffectorSubsystem.isPresent() && elevatorSubsystem.isPresent())
        {
            SK25EndEffector endEffector = endEffectorSubsystem.get();
            SK25Elevator elevator = elevatorSubsystem.get();

            // endEffector Position Buttons
            zeroPositionButton.onTrue(new ScoreCommandGroup(kZeroPosition, elevator, kZeroPositionAngle, endEffector));
            troughPositionButton.onTrue(new ScoreCommandGroup(kTroughPosition, elevator, kTroughPositionAngle, endEffector));
            lowPositionButton.onTrue(new ScoreCommandGroup(kLowPosition, elevator, kMidLowPositionAngle, endEffector));
            midPositionButton.onTrue(new ScoreCommandGroup(kMidPosition, elevator, kMidLowPositionAngle, endEffector));
            intakePositionButton.onTrue(new EndEffectorButtonCommand(kIntakePositionAngle, endEffector));
            topPositionButton.onTrue(new ScoreCommandGroup(kTopPosition, elevator, kTopPositionAngle, endEffector));                         
        }
    }
}