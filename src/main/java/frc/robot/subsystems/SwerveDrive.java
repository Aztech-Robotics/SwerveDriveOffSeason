package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveMode;

public class SwerveDrive extends SubsystemBase {
  public static SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
    //FrontLeft
    new Translation2d(Constants.trackWidth/2, Constants.wheelBase/2),
    //FrontRight
    new Translation2d(Constants.trackWidth/2, -Constants.wheelBase/2),
    //RearLeft
    new Translation2d(-Constants.trackWidth/2, Constants.wheelBase/2),
    //RearRight 
    new Translation2d(-Constants.trackWidth/2, -Constants.wheelBase/2)
  );
  private SwerveModule[] modules = new SwerveModule[4];
  private ChassisSpeeds desiredChassisSpeeds = null;

  private SwerveMode swerveMode = SwerveMode.Nothing;
  private SwerveDriveOdometry swerveDriveOdometry = null;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private ShuffleboardTab tabDrive = Shuffleboard.getTab("DriveData"); 

  public SwerveDrive() {
    modules[0] = new SwerveModule(Constants.id_drive_fLeft, Constants.id_steer_fLeft, Constants.id_canCoder_fLeft, Constants.offset_fLeft);
    modules[1] = new SwerveModule(Constants.id_drive_fRight, Constants.id_steer_fRight, Constants.id_canCoder_fRight, Constants.offset_fRight);
    modules[2] = new SwerveModule(Constants.id_drive_bLeft, Constants.id_steer_bLeft, Constants.id_canCoder_bLeft, Constants.offset_bLeft);
    modules[3] = new SwerveModule(Constants.id_drive_bRight, Constants.id_steer_bRight, Constants.id_canCoder_bRight, Constants.offset_bRight);
    setCoastMode();
    outputTelemetry();
  }

  @Override
  public void periodic() {
    switch (swerveMode){
      case Nothing:
      break; 
      //Update outputs in Meters Per Second
      case OpenLoopWithVelocity:
        SwerveModuleState[] moduleStatesArray;
        if (desiredChassisSpeeds != null) {
          moduleStatesArray = swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(moduleStatesArray, Constants.maxDriveVel);
          setModulesStatesWithVelocity(moduleStatesArray);
        }
        desiredChassisSpeeds = null;
      break;
      //Update outputs in Percent
      case OpenLoopWithVoltage:
      break;
      //Update Odometry
      case Trajectory:
        updateOdometry();
      break;
    }
  }

  public void setMode (SwerveMode swerveMode){
    this.swerveMode = swerveMode;
  }

  public void setBrakeMode (){
    for (SwerveModule swerveModule : modules){
      swerveModule.setIdleMode(IdleMode.kBrake);
    }
  }

  public void setCoastMode (){
    for (SwerveModule swerveModule : modules){
      swerveModule.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public SwerveModuleState[] getModulesStates (){
    SwerveModuleState[] swerveModulesStates = new SwerveModuleState[4];
    for (int i=0; i < modules.length; i++){
      swerveModulesStates[i] = modules[i].getModuleState();
    }
    return swerveModulesStates;
  }
  
  public void setModulesStatesWithVelocity (SwerveModuleState[] swerveModulesModuleStates){
    for (int i=0; i < modules.length; i++){
      modules[i].setModuleState(SwerveModuleState.optimize(swerveModulesModuleStates[i], modules[i].getCanCoderAngle()));
    }
  }

  public void setModulesStatesWithVoltage (SwerveModuleState[] swerveModulesModuleStates){
    for (int i=0; i < modules.length; i++){
      modules[i].setModuleState(SwerveModuleState.optimize(swerveModulesModuleStates[i], modules[i].getCanCoderAngle()));
    }
  }
  
  public SwerveModulePosition[] getModulesPosition(){
    SwerveModulePosition[] modulesPosition = new SwerveModulePosition[4];
    for (int i=0; i < modules.length; i++){
      modulesPosition[i] = modules[i].getModulePosition();
    }
    return modulesPosition;
  }
  
  public void setModulesPosition(SwerveModulePosition[] modulesPosition){
    for (int i=0; i < modules.length; i++){
      modules[i].setModulePosition(modulesPosition[i]);
    }
  }
  
  public Pose2d getCurrentPose (){
    return swerveDriveOdometry.getPoseMeters();
  }
  
  public void setCurrentPose (Pose2d pose){
    swerveDriveOdometry.resetPosition(getGyroAngle(), getModulesPosition(), pose);
  }
  
  public void setDesiredChassisSpeeds (ChassisSpeeds chassisSpeeds){
    desiredChassisSpeeds = chassisSpeeds;
  }

  public void resetChassisPosition (){
    resetGyroAngle();
    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroAngle(), getModulesPosition());
  }
  
  public void resetGyroAngle (){
    gyro.reset();
  }
  
  public Rotation2d getGyroAngle (){
    double angle = Math.IEEEremainder(-gyro.getAngle(), 360);
    Rotation2d gyroRotation = Rotation2d.fromDegrees(angle);
    return gyroRotation;
  }

  public Rotation2d getGyroAngle2 (){
    Rotation2d gyroRotation = gyro.getRotation2d();
    double mod = gyroRotation.getDegrees() % 360;
    if (mod != 0){
      if (mod > 0){
        gyroRotation = Rotation2d.fromDegrees(Math.abs(mod));
      } else {
        gyroRotation = Rotation2d.fromDegrees(Math.abs(360 - mod));
      }
    }
    return gyroRotation;
  }
  
  public void updateOdometry (){
    swerveDriveOdometry.update(getGyroAngle(), getModulesPosition());
  }

  public void restartPositionSteerMotor (){
    for (SwerveModule swerveModule : modules){
      swerveModule.setAngleCanCoderToPositionMotor();
    }
  }

  public void outputTelemetry (){
    tabDrive.addDouble("GyroAngle", ()->{return getGyroAngle().getDegrees();}); 
  }
  
  @Override
  public void simulationPeriodic() {
  }
}
