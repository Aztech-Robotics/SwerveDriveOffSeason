package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator = null;
  private ShuffleboardTab tabSwerve = Shuffleboard.getTab("SwerveData");
  private Limelight limelight = new Limelight();

  public SwerveDrive() {
    modules[0] = new SwerveModule(Constants.id_drive_fLeft, Constants.id_steer_fLeft, Constants.id_canCoder_fLeft, Constants.offset_fLeft);
    modules[1] = new SwerveModule(Constants.id_drive_fRight, Constants.id_steer_fRight, Constants.id_canCoder_fRight, Constants.offset_fRight);
    modules[2] = new SwerveModule(Constants.id_drive_bLeft, Constants.id_steer_bLeft, Constants.id_canCoder_bLeft, Constants.offset_bLeft);
    modules[3] = new SwerveModule(Constants.id_drive_bRight, Constants.id_steer_bRight, Constants.id_canCoder_bRight, Constants.offset_bRight);
    setCoastMode();
    outputTelemetry();
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, 
    getGyroAngle(), 
    getModulesPosition(), 
    new Pose2d(), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0, 0, 0), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0, 0, 0));
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
        if (desiredChassisSpeeds != null) {
          moduleStatesArray = swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(moduleStatesArray, Constants.maxDriveVel);
          setModulesStatesWithVoltage(moduleStatesArray);
        }
        desiredChassisSpeeds = null;
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
      modules[i].setModuleStateWithVelocity(SwerveModuleState.optimize(swerveModulesModuleStates[i], modules[i].getCanCoderAngle()));
    }
  }

  public void setModulesStatesWithVoltage (SwerveModuleState[] swerveModulesModuleStates){
    for (int i=0; i < modules.length; i++){
      modules[i].setModuleStateWithVoltage(SwerveModuleState.optimize(swerveModulesModuleStates[i], modules[i].getCanCoderAngle()));
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
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }
  
  public void setCurrentPose (Pose2d pose){
    swerveDrivePoseEstimator.resetPosition(getGyroAngle(), getModulesPosition(), pose);
  }
  
  public void setDesiredChassisSpeeds (ChassisSpeeds chassisSpeeds){
    desiredChassisSpeeds = chassisSpeeds;
  }

  public void resetChassisPosition (Pose2d initialPose){ 
    resetGyroAngle();
    for (SwerveModule module : modules){
      module.setPositionSpeedMotor(0);
    }
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, 
    getGyroAngle(), 
    getModulesPosition(), 
    initialPose, 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0, 0, 0), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0, 0, 0));
  }
  
  public void resetGyroAngle (){
    gyro.reset();
  }

  
  public Rotation2d getGyroAngle (){
    double angle = Math.IEEEremainder(-gyro.getAngle(), 360);
    Rotation2d gyroRotation = Rotation2d.fromDegrees(angle);
    return gyroRotation;
  }
  
  public void updateOdometry (){
    if (limelight.sawTag()){
      Alliance alliance = DriverStation.getAlliance();
      switch (alliance){
        case Blue:
          swerveDrivePoseEstimator.addVisionMeasurement(limelight.getBotPoseBlueAlliance(), Timer.getFPGATimestamp() - (limelight.getLatencyPipeline()/1000.0) - (limelight.getLatencyCapture()/1000.0));
        break;
        case Red:
          swerveDrivePoseEstimator.addVisionMeasurement(limelight.getBotPoseRedAlliance(), Timer.getFPGATimestamp() - (limelight.getLatencyPipeline()/1000.0) - (limelight.getLatencyCapture()/1000.0));
        break;
        case Invalid:
        break;
      }
    } else {
      swerveDrivePoseEstimator.update(getGyroAngle(), getModulesPosition());
    }
  }

  public void restartPositionSteerMotor (){
    for (SwerveModule swerveModule : modules){
      swerveModule.setAngleCanCoderToPositionMotor();
    }
  }

  public void outputTelemetry (){
    tabSwerve.addDouble("X Pose Odometry", ()->{return getCurrentPose().getX();}).withPosition(8, 0);
    tabSwerve.addDouble("Y Pose Odometry", ()->{return getCurrentPose().getY();}).withPosition(9, 0);
    tabSwerve.addDouble("GyroAngle", ()->{return getGyroAngle().getDegrees();}).withPosition(8, 1); 
    tabSwerve.addDouble("TAG ID", ()->{return limelight.getTagID();}).withPosition(9, 1);
  }
}
