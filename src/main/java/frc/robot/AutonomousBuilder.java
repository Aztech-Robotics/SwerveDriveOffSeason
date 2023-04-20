package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class AutonomousBuilder {
    private final SwerveDrive m_swerveDrive;
    private AutoBase m_autoBase;

    AutonomousBuilder (SwerveDrive swerveDrive){
        m_swerveDrive = swerveDrive;
    }

    public Command createCommand (AutoBase autoBase){
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_swerveDrive::getCurrentPose, 
            m_swerveDrive::setCurrentPose, 
            SwerveDrive.swerveDriveKinematics, 
            new PIDConstants(0, 0, 0), 
            new PIDConstants(0, 0, 0), 
            m_swerveDrive::setModulesStates, 
            m_autoBase.getEventMap(), 
            true, 
            m_swerveDrive
        );
        return autoBuilder.followPathWithEvents(m_autoBase.getTrajectory()); 
    }
}
