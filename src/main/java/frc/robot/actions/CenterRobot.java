package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public class CenterRobot extends CommandBase {
  private final SwerveDrive m_SwerveDrive;
  private final Limelight m_Limelight;
  public CenterRobot(SwerveDrive _SwerveDrive, Limelight _Limelight) {
    m_SwerveDrive = _SwerveDrive;
    m_Limelight = _Limelight;
    addRequirements(m_SwerveDrive);
  }

  @Override
  public void initialize() {
    m_Limelight.setPipelineByGeneralMode();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
