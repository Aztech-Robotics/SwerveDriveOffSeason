package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoBase;

public class Auto1_SecondTrajectory implements AutoBase {
    public Auto1_SecondTrajectory (){
        
    }

    @Override public HashMap<String, Command> getEventMap (){
        return null;
    };

    @Override public PathPlannerTrajectory getTrajectory (){
        return null;
    }; 
}
