package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK25EndEffector;

import static frc.robot.Konstants.EndEffectorConstants.kArmSpeed;
import static frc.robot.Konstants.EndEffectorConstants.kRollerSpeed;
import static frc.robot.Ports.OperatorPorts.kIntakeAxis;
import static frc.robot.Ports.OperatorPorts.kExtakeAxis;


public class EndEffectorRollerCommand extends Command {

    private final SK25EndEffector Subsystem;

    private boolean intake;

    private double intakeAxis;
    private double extakeAxis;

    private double intakeSpeed;
    private double extakeSpeed;

    public EndEffectorRollerCommand(boolean intake, SK25EndEffector Subsystem)
    {
        this.Subsystem = Subsystem;
        this.intake = intake;
    }

    public void initialize()
    {
       if(intake == true)
       {
        intakeAxis = kIntakeAxis.getFilteredAxis();
        intakeSpeed = -kRollerSpeed * intakeAxis;   
        Subsystem.runRoller(intakeSpeed);
       }
       else if(intake == false)
       {
        extakeAxis = kExtakeAxis.getFilteredAxis();
        extakeSpeed = kRollerSpeed * extakeAxis;
        Subsystem.runRoller(kRollerSpeed);
       }
    }

    public boolean isFinished()
    {
        return true;
    }
    
}