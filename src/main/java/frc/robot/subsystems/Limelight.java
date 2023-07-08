package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.GeneralMode;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TypePipeline;

public class Limelight {
  private TypePipeline activePipeline;

  public Limelight() {
    setPipelineByGeneralMode();
  }

  public double getTagID (){
    return LimelightHelpers.getFiducialID(null);
  }

  public boolean sawTag (){
    return getTagID() != -1? true : false; 
  }

  public Pose2d getBotPoseBlueAlliance (){
    return LimelightHelpers.getBotPose2d_wpiBlue(null);
  }

  public Pose2d getBotPoseRedAlliance (){
    return LimelightHelpers.getBotPose2d_wpiRed(null);
  }

  public double getX (){
    return LimelightHelpers.getTX(null);
  }

  public double getY (){
    return LimelightHelpers.getTY(null);
  }

  public double getLatencyPipeline (){
    return LimelightHelpers.getLatency_Pipeline(null);
  }

  public double getLatencyCapture (){
    return LimelightHelpers.getLatency_Capture(null);
  }

  public void setPipeline(TypePipeline pipeline){
    activePipeline = pipeline;
    switch (pipeline){
      case RetroflectiveTape:
      LimelightHelpers.setPipelineIndex(null, 0);
      break;
      case AprilTag:
      LimelightHelpers.setPipelineIndex(null, 1);
      break;
    }
  }

  public void setPipelineByGeneralMode (){
    switch (GeneralMode.getInstance().getMode()){
      case Cone:
      setPipeline(TypePipeline.RetroflectiveTape);
      break;
      case Cube:
      setPipeline(TypePipeline.AprilTag);
      break;
    }
  }
}