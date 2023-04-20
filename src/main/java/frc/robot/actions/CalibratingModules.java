package frc.robot.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class CalibratingModules extends CommandBase {
  private final SwerveDrive m_SwerveDrive;
  public CalibratingModules(SwerveDrive _SwerveDrive) {
    m_SwerveDrive = _SwerveDrive;
    addRequirements(m_SwerveDrive);
  }

  @Override
  public void initialize() {
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
