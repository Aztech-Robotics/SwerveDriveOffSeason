package frc.robot.actions;

import frc.robot.Constants;
import frc.robot.Constants.SwerveMode;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FieldOrientedDrive extends CommandBase {
  private final SwerveDrive m_SwerveDrive;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(5);
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
    m_SwerveDrive.restartPositionSteerMotor();
    SwerveModulePosition[] zeroPositions = new SwerveModulePosition[4];
    for (SwerveModulePosition modulePosition : zeroPositions){
      modulePosition = new SwerveModulePosition();
      modulePosition.compareTo(modulePosition);
    }
    m_SwerveDrive.setModulesPosition(zeroPositions);
  }

  @Override
  public void execute() {
    m_SwerveDrive.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      translateXRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble() * Constants.maxDriveVel, 0.1)),
      translateYRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble() * Constants.maxDriveVel, 0.1)),
      rotationRateLimiter.calculate(MathUtil.applyDeadband(rotationSupplier.getAsDouble() * Constants.maxAngVel, 0.1)),
      m_SwerveDrive.getGyroAngle()
    ));
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveDrive.setDesiredChassisSpeeds(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
