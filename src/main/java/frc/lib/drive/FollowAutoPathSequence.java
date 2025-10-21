package frc.lib.drive;

import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;

public class FollowAutoPathSequence {

    public static Command NewPathSequenceCommand(String autoName) {
        List<PathPlannerPath> pathSequence = generatePathSequence(autoName);

        if(pathSequence != null) {
            List<Command> followPathCommands = generateFollowPathSequence(pathSequence);
            return Commands.sequence(followPathCommands.toArray(new Command[0]));
        }

        return Commands.none();
    }

    private static List<PathPlannerPath> generatePathSequence(String autoName) {
        try {
            List<PathPlannerPath> extractedPathSequence = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            return extractedPathSequence;
        }
        catch (Exception e) {
            if(e instanceof IOException) {
                System.out.println("Auto file not found");
            }
            else if(e instanceof ParseException) {
                System.out.println("JSON could not be parsed");
            }
        }

        return null;
    }

    private static List<Command> generateFollowPathSequence(List<PathPlannerPath> pathSequence) {
        List<Command> followPathCommands = new ArrayList<>();

        for(PathPlannerPath path : pathSequence) {
            followPathCommands.add(AutoBuilder.followPath(path));
        }
        followPathCommands.set(0, AutoBuilder.pathfindThenFollowPath(pathSequence.get(0), kDefaultPathfindingConstraints));

        return followPathCommands;
    }
}
