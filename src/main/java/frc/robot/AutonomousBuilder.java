package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveMode;
import frc.robot.subsystems.SwerveDrive;

public class AutonomousBuilder {
    private final SwerveDrive m_swerveDrive;
    private AutoBase m_autoBase;

    AutonomousBuilder (SwerveDrive swerveDrive){
        m_swerveDrive = swerveDrive;
    }

    public Command createCommand (AutoBase autoBase){
        m_autoBase = autoBase;
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_swerveDrive::getCurrentPose, 
            m_swerveDrive::setCurrentPose, 
            SwerveDrive.swerveDriveKinematics, 
            new PIDConstants(5, 0, 0), 
            new PIDConstants(1, 0, 0), 
            m_swerveDrive::setModulesStatesWithVelocity, 
            m_autoBase.getEventMap(), 
            true, 
            m_swerveDrive
        );
        m_swerveDrive.resetChassisPosition(); 
        m_swerveDrive.setCurrentPose(m_autoBase.getTrajectory().getInitialHolonomicPose());
        m_swerveDrive.setMode(SwerveMode.Trajectory);
        return autoBuilder.followPathWithEvents(m_autoBase.getTrajectory()); 
    }
}
