package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoBase;

public class AutoPath3 implements AutoBase {
    private PathPlannerTrajectory trajectory = PathPlanner.loadPath("AutoPath3", new PathConstraints(4, 3));
    public AutoPath3 (){
    }

    @Override public HashMap<String, Command> getEventMap (){
        return null;
    }

    @Override public PathPlannerTrajectory getTrajectory (){
        return trajectory;
    }

    @Override
    public boolean isFirstPath (){
        return false;
    }
}
