package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double trackWidth = Units.inchesToMeters(20);
  public static final double wheelBase = Units.inchesToMeters(20);
  public static final double wheelDiameter = 0.1;
  public static final double max_rpm_spark = 5676;
  public static final double drive_gear_ratio = 6.75;
  public static final double steer_gear_ratio = 12.8;
  
  //MetersPerSecond
  public static final double maxDriveVel = ((max_rpm_spark / 60) / drive_gear_ratio) * Math.PI * wheelDiameter; 
  //RadiansPerSecond
  public static final double maxAngVel = ((max_rpm_spark / 60) / steer_gear_ratio) * Math.PI * 2; 

  public static final double drivePositionCoefficient = (Math.PI * wheelDiameter) / drive_gear_ratio;
  public static final double driveVelocityCoefficient = drivePositionCoefficient / 60;
  public static final double steerPositionCoefficient = (Math.PI * 2) / steer_gear_ratio;
  public static final double steerVelocityCoefficient = steerPositionCoefficient / 60; 
  
  public static final int id_drive_fLeft = 1;
  public static final int id_steer_fLeft = 2;
  public static final int id_canCoder_fLeft = 3;
  public static final Rotation2d offset_fLeft = Rotation2d.fromDegrees(61);

  public static final int id_drive_fRight = 4;
  public static final int id_steer_fRight = 5;
  public static final int id_canCoder_fRight = 6;
  public static final Rotation2d offset_fRight = Rotation2d.fromDegrees(5);

  public static final int id_drive_bLeft = 7;
  public static final int id_steer_bLeft = 8;
  public static final int id_canCoder_bLeft = 9;
  public static final Rotation2d offset_bLeft = Rotation2d.fromDegrees(51);

  public static final int id_drive_bRight = 10;
  public static final int id_steer_bRight = 11;
  public static final int id_canCoder_bRight = 12;
  public static final Rotation2d offset_bRight = Rotation2d.fromDegrees(274); 

  public static final double kp_steerController = 0.8;
  public static final double ki_steerController = 0.0;
  public static final double kd_steerController = 1;
  public static final double kf_steerController = 0.0;
  public static final double kIz_steerController = 0.0;

  public static final double kp_speedController = 0.0004;
  public static final double ki_speedController = 0.00002;
  public static final double kd_speedController = 0.0;
  public static final double kf_speedController = 0.0;
  public static final double kIz_speedController = 0.0;

  public static enum SwerveMode {
    Nothing,
    OpenLoopWithVoltage,
    OpenLoopWithVelocity,
    Trajectory
  }

  public static enum GeneralModeEnum {
    Cone,
    Cube
  }

  public static enum TypePipeline {
    AprilTag,
    RetroflectiveTape
  }

  public static final PathConstraints autoConstraints = new PathConstraints(5.5, 5);
}
