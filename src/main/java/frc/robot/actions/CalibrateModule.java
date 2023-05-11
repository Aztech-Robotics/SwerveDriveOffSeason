package frc.robot.actions;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class CalibrateModule extends CommandBase {
  private SwerveDrive swerveDrive;
  private DoubleSupplier stickSpeed, stickSteer;
  private int module;
  public CalibrateModule(SwerveDrive swerveDrive, int module, DoubleSupplier stickSpeed, DoubleSupplier stickSteer) {
    this.swerveDrive = swerveDrive;
    this.stickSpeed = stickSpeed;
    this.stickSteer = stickSteer; 
    this.module = module;
    addRequirements(this.swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveDrive.setOnlyOneModule(module, stickSpeed.getAsDouble(), MathUtil.applyDeadband(stickSteer.getAsDouble(), 0.1));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
