package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoBase {
    public HashMap<String, Command> getEventMap ();
    public PathPlannerTrajectory getTrajectory (); 
}
