package frc.robot.commands.GoToSource;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class GoToProcessorSourceFromSE extends Command {
    private boolean valid;
    private Command pathFindingCommand;
    
    public Command GoToProcessorSideSourceFromSE() {
        PathConstraints constraints = new PathConstraints(3, 3, 540, 720);

        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("ProcessorSideSourceSEVision");
            valid = true;
        } catch (FileVersionException e) {
            valid = false;
            path = null;

            e.printStackTrace();
        } catch (IOException e) {
            valid = false;
            path = null;

            e.printStackTrace();
        } catch (ParseException e) {
            valid = false;
            path = null;

            e.printStackTrace();
        }

        if(valid) {
            pathFindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
        }
        else {
            pathFindingCommand = Commands.waitSeconds(15);
        }

        return pathFindingCommand;
    }
}
