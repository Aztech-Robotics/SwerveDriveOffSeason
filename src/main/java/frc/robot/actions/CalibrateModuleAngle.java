package frc.robot.actions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class CalibrateModuleAngle extends InstantCommand {
  private SwerveDrive swerveDrive;
  private int module;
  private Rotation2d angle;

  public CalibrateModuleAngle(SwerveDrive swerveDrive, int module, Rotation2d targetAngle) {
    this.swerveDrive = swerveDrive;
    this.module = module;
    this.angle = targetAngle;
  }

  @Override
  public void initialize() {
    swerveDrive.setAngleOnlyOneModule(module, angle);
  }
}
