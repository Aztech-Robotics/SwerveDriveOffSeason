package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoBase;

public class FullAuto implements AutoBase {
    public FullAuto (){
    }

    @Override public HashMap<String, Command> getEventMap (){
        return null;
    }

    @Override public PathPlannerTrajectory getTrajectory (){
        return null;
    }
}
