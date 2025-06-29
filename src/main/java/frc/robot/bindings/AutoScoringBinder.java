package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kAlignToReef;
import static frc.robot.Ports.DriverPorts.kLeftReef;
import static frc.robot.Ports.DriverPorts.kRightReef;
import static frc.robot.Ports.OperatorPorts.kAutoScoringToggle;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToReefTag.Target;
import frc.robot.commands.commandGroups.AlignToReefComboTeleop;
import frc.robot.commands.commandGroups.AutoScoreCombo;
import frc.robot.commands.commandGroups.AutoScoreCombo.ScoreType;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SK25EndEffector;
import frc.robot.subsystems.SKSwerve;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.subsystems.vision.SK25Vision.DriveToPose;
import frc.robot.subsystems.vision.SK25Vision.RotateToPose;

public class AutoScoringBinder implements CommandBinder{
    // Subsystem optionals go here

    Trigger autoScoringToggle;
    Trigger scoreLeftBranch;
    Trigger scoreRightBranch;
    Trigger grabReefAlgae;
    Trigger autoScoring;

    Optional<SK25Vision> m_visionContainer; 
    Optional<SKSwerve> m_swerveContainer;
    Optional<CoralSubsystem> m_elevatorContainer;
    Optional<SK25EndEffector> m_endEffectorContainer;

    public AutoScoringBinder(
        Optional<SK25Vision> m_visionContainer, 
        Optional<SKSwerve> m_swerveContainer,
        Optional<CoralSubsystem> m_elevatorContainer,
        Optional<SK25EndEffector> m_endEffectorContainer) 
    {
        autoScoringToggle = kAutoScoringToggle.button;
        autoScoring = new Trigger(() -> true);
        scoreLeftBranch = kLeftReef.button;
        scoreRightBranch = kRightReef.button;
        grabReefAlgae = kAlignToReef.button;

        this.m_visionContainer = m_visionContainer;
        this.m_swerveContainer = m_swerveContainer;
        this.m_elevatorContainer = m_elevatorContainer;
        this.m_endEffectorContainer = m_endEffectorContainer;
    }

    @Override
    public void bindButtons() {
        if(
            m_visionContainer.isPresent() && m_swerveContainer.isPresent() 
            && m_elevatorContainer.isPresent() && m_endEffectorContainer.isPresent()
        ) 
        {

            SK25Vision m_vision = m_visionContainer.get();
            SKSwerve m_swerve = m_swerveContainer.get();
            CoralSubsystem m_elevator = m_elevatorContainer.get();
            SK25EndEffector m_endEffector = m_endEffectorContainer.get();
            
            // By negating the preexisting Trigger, it essentially flips its boolean for when you're Trigger-chaining
            autoScoringToggle.onTrue(new InstantCommand(() -> {autoScoring = autoScoring.negate();}));

            scoreLeftBranch.and(autoScoring).whileTrue(
                new AutoScoreCombo(
                    Target.LEFT, 
                    ScoreType.CORAL, 
                    m_swerve, 
                    m_vision, 
                    m_elevator, 
                    m_endEffector)
            );
            scoreRightBranch.and(autoScoring).whileTrue(
                new AutoScoreCombo(
                    Target.RIGHT, 
                    ScoreType.CORAL, 
                    m_swerve, 
                    m_vision, 
                    m_elevator, 
                    m_endEffector)
            );
            grabReefAlgae.and(autoScoring).whileTrue(
                new AutoScoreCombo(
                    Target.CENTER, 
                    ScoreType.ALGAE, 
                    m_swerve, 
                    m_vision, 
                    m_elevator, 
                    m_endEffector)
            );

            // Resort back to normal vision behaviors if auto scoring is disabled
            scoreLeftBranch.and(autoScoring.negate()).whileTrue(
                new AlignToReefComboTeleop(
                    Target.LEFT, 
                    DriveToPose.getConfig(), 
                    RotateToPose.getConfig(), 
                    m_vision, 
                    m_swerve)
            );
            scoreRightBranch.and(autoScoring.negate()).whileTrue(
                new AlignToReefComboTeleop(
                    Target.RIGHT, 
                    DriveToPose.getConfig(), 
                    RotateToPose.getConfig(), 
                    m_vision, 
                    m_swerve)
            );
            grabReefAlgae.and(autoScoring.negate()).whileTrue(
                new AlignToReefComboTeleop(
                    Target.CENTER, 
                    DriveToPose.getConfig(),
                    RotateToPose.getConfig(),
                    m_vision,
                    m_swerve)
            );
        }
    }

}
