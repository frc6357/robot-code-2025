package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.commands.AlignToReefTag.Target;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.commands.AlignToReefTag;
import frc.robot.subsystems.SKSwerve;


public class AlignToReefComboAuton extends SequentialCommandGroup {
    /**
     * Basically the same as {@link AlignToReefComboTeleop} but with no WaitCommand
     * @param t The target position of the reef to align to
     * @param xyConfig The vision CommandConfig for linear movement
     * @param rotConfig The vision CommandConfig for rotational movement
     * @param m_vision The vision subsystem to reference detection methods
     * @param m_swerve The swerve subsystem to add requirements
     */
    public AlignToReefComboAuton(
        Target t, 
        MultiLimelightCommandConfig xyConfig,
        MultiLimelightCommandConfig rotConfig,
        SK25Vision m_vision,
        SKSwerve m_swerve) 
    {
        addRequirements(m_swerve);
        addCommands(
            new AlignToReefTag(Target.BACK, xyConfig, rotConfig, m_vision, m_swerve),
            new AlignToReefTag(t, xyConfig, rotConfig, m_vision, m_swerve)
        );
    }
}
