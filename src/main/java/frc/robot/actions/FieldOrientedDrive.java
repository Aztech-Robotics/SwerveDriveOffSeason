package frc.robot.actions;

import frc.robot.Constants;
import frc.robot.Constants.SwerveMode;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FieldOrientedDrive extends CommandBase {
  private final SwerveDrive m_SwerveDrive;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  public FieldOrientedDrive(SwerveDrive _SwerveDrive, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    m_SwerveDrive = _SwerveDrive;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(m_SwerveDrive);
  }

  @Override
  public void initialize() {
    m_SwerveDrive.setMode(SwerveMode.OpenLoopWithVelocity);
  }

  @Override
  public void execute() {
    m_SwerveDrive.setDesiredChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        MathUtil.applyDeadband(translationXSupplier.getAsDouble(), 0.2) * Constants.maxDriveVel,
        MathUtil.applyDeadband(translationYSupplier.getAsDouble(), 0.2) * Constants.maxDriveVel,
        MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.2) * Constants.maxAngVel,
        m_SwerveDrive.getGyroAngle()
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveDrive.setDesiredChassisSpeeds(new ChassisSpeeds());
    m_SwerveDrive.setMode(SwerveMode.Nothing);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
