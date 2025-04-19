package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.commands.AlignToAnyReefTag.AnyTarget;
import frc.robot.subsystems.vision.SK25Vision.MultiLimelightCommandConfig;
import frc.robot.subsystems.vision.SK25Vision;
import frc.robot.commands.AlignToAnyReefTag;
import frc.robot.subsystems.SKSwerve;


public class AlignToAnyReefComboTeleop extends SequentialCommandGroup {
    /**
     * Creates a sequential group that aligns the robot to a far, centered position
     * relative to the closest reef face, then approaches the specific branch targetted
     * by the constructor
     * @param t The target position of the reef to align to
     * @param xyConfig The vision CommandConfig for linear movement
     * @param rotConfig The vision CommandCOnfig for rotational movement
     * @param m_vision The vision subsystem to reference detection methods
     * @param m_swerve The swerve subsystem to add requirements
     */
    public AlignToAnyReefComboTeleop(
        AnyTarget t, 
        MultiLimelightCommandConfig xyConfig,
        MultiLimelightCommandConfig rotConfig,
        SK25Vision m_vision,
        SKSwerve m_swerve) 
    {
        addRequirements(m_swerve);
        addCommands(
            // new AlignToReefTag(Target.BACK, xyConfig, rotConfig, m_vision, m_swerve),
            // new WaitCommand(0.15),
            new AlignToAnyReefTag(t, xyConfig, rotConfig, m_vision, m_swerve).withTimeout(3)
        );
    }
}
