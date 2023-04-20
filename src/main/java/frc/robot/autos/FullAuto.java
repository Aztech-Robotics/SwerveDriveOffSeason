package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoBase;

public class FullAuto implements AutoBase {
    private ArrayList<PathPlannerTrajectory> pathgroup;
    public FullAuto (){
        pathgroup.add(PathPlanner.loadPath("PickUpFirstPiece", new PathConstraints(5.5, 5)));
        pathgroup.add(PathPlanner.loadPath("LeaveSecondPiece", new PathConstraints(5.5, 5)));
        pathgroup.add(PathPlanner.loadPath("CSFromL2ndPiece", new PathConstraints(5.5, 5)));
    }

    @Override public HashMap<String, Command> getEventMap (){
        return null;
    }

    @Override public PathPlannerTrajectory getTrajectory (){
        return null;
    }
}
