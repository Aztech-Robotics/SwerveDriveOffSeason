package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GeneralMode;
import frc.robot.Constants.TypePipeline;

public class Limelight extends SubsystemBase {
  private TypePipeline activePipeline;

  public Limelight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    setPipelineByGeneralMode();
  }

  @Override
  public void periodic() {
  }

  public double getTargetXOffset (){
    return 0;
  }

  public double getTargetYOffset (){
    return 0;
  }

  public void setPipeline(TypePipeline pipeline){
    activePipeline = pipeline;
    switch (pipeline){
      case RetroflectiveTape:
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      break;
      case AprilTag:
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
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
