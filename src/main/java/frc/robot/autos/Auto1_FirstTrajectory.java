package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoBase;

public class Auto1_FirstTrajectory implements AutoBase {
    private PathPlannerTrajectory trajectory;
    public Auto1_FirstTrajectory (){
        trajectory = PathPlanner.loadPath("PickUpFirstPiece", new PathConstraints(5.5, 5));
    }

    @Override public HashMap<String, Command> getEventMap (){
        return null;
    }

    @Override public PathPlannerTrajectory getTrajectory (){
        return trajectory;
    }
}
