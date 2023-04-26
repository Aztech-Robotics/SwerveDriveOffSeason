package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SwerveModule {
    private CANSparkMax speedMotor;
    private RelativeEncoder encoder_speedMotor;
    private SparkMaxPIDController controller_speedMotor;
    private CANSparkMax steerMotor;
    private RelativeEncoder encoder_steerMotor;
    private SparkMaxPIDController controller_steerMotor;
    private CANCoder cancoder_steerMotor;
    private Rotation2d steerOffset;
    private ShuffleboardTab tabDrivePositions = Shuffleboard.getTab("MotorsData");

    public SwerveModule (int id_speedMotor, int id_steerMotor, int id_steerCanCoder, Rotation2d steerOffset){
        this.steerOffset = steerOffset;
        speedMotor = new CANSparkMax(id_speedMotor, MotorType.kBrushless);
        encoder_speedMotor = speedMotor.getEncoder();
        encoder_speedMotor.setPositionConversionFactor(Constants.drivePositionCoefficient);
        encoder_speedMotor.setVelocityConversionFactor(Constants.driveVelocityCoefficient);
        controller_speedMotor = speedMotor.getPIDController();
        controller_speedMotor.setFeedbackDevice(encoder_speedMotor);
        controller_speedMotor.setP(Constants.kp_speedController, 0);
        controller_speedMotor.setI(Constants.ki_speedController, 0);
        controller_speedMotor.setD(Constants.kd_speedController, 0);
        controller_speedMotor.setFF(Constants.kf_speedController, 0);
        controller_speedMotor.setIZone(Constants.kIz_speedController, 0);

        steerMotor = new CANSparkMax(id_steerMotor, MotorType.kBrushless);
        encoder_steerMotor = steerMotor.getEncoder();
        encoder_steerMotor.setPositionConversionFactor(Constants.steerPositionCoefficient);
        encoder_steerMotor.setVelocityConversionFactor(Constants.steerVelocityCoefficient);
        controller_steerMotor = steerMotor.getPIDController();
        controller_steerMotor.setFeedbackDevice(encoder_steerMotor);
        controller_steerMotor.setP(Constants.kp_steerController, 0);
        controller_steerMotor.setI(Constants.ki_steerController, 0);
        controller_steerMotor.setD(Constants.kd_steerController, 0);
        controller_steerMotor.setFF(Constants.kf_steerController, 0);
        controller_steerMotor.setIZone(Constants.kIz_steerController, 0);
        //controller_steerMotor.setPositionPIDWrappingEnabled(true);
        //controller_steerMotor.setPositionPIDWrappingMinInput(0);
        //controller_steerMotor.setPositionPIDWrappingMaxInput(Math.PI * 2);

        cancoder_steerMotor = new CANCoder(id_steerCanCoder);
        cancoder_steerMotor.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        cancoder_steerMotor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        cancoder_steerMotor.configMagnetOffset(steerOffset.getDegrees());

        outputTelemetry(); 
    }
    
    public void voltageSpeedMotor (double output){
        controller_speedMotor.setReference(output, CANSparkMax.ControlType.kDutyCycle);
    }
    
    public void velocitySpeedMotor (double velocity){
        controller_speedMotor.setReference(velocity, CANSparkMax.ControlType.kVelocity); 
    }
    
    public double getVelocitySpeedMotor (){
        return encoder_speedMotor.getVelocity();
    }
    
    public void setPositionSpeedMotor (double position){
        encoder_speedMotor.setPosition(position);
    }
    
    public double getPositionSpeedMotor (){
        return encoder_speedMotor.getPosition();
    }
    
    public void voltageSteerMotor (double output){
        controller_steerMotor.setReference(output, CANSparkMax.ControlType.kDutyCycle);
    }

    public Rotation2d getCanCoderAngle (){
        return Rotation2d.fromDegrees(cancoder_steerMotor.getAbsolutePosition());
    }

    public void setAngleCanCoderToPositionMotor (){
        encoder_steerMotor.setPosition(getCanCoderAngle().getRadians());
    }

    public Rotation2d getSteerMotorAngle (){
        double adjustedAngle;
        double moduleAngle = encoder_steerMotor.getPosition() % (Math.PI * 2);
        if (Math.abs(moduleAngle) > 0){
            adjustedAngle = moduleAngle;
        }
        else {
            adjustedAngle = encoder_steerMotor.getPosition();
        }
        if (adjustedAngle < 0){
            adjustedAngle += Math.PI * 2;
        }
        return Rotation2d.fromRadians(adjustedAngle);
    }
    
    public void angleSteerMotor (Rotation2d angle){
        Rotation2d errorAngle = constraintAngle(angle).minus(constraintAngle(getSteerMotorAngle()));
        controller_steerMotor.setReference(encoder_steerMotor.getPosition() + errorAngle.getRadians(), CANSparkMax.ControlType.kPosition);
    }
    
    public Rotation2d constraintAngle (Rotation2d angle){
        double angleRadians = angle.getRadians(); 
        if (angleRadians > Math.PI){
            angleRadians -= Math.PI * 2;
        }
        if (angleRadians < -Math.PI){
            angleRadians += Math.PI * 2;
        }
        return Rotation2d.fromRadians(angleRadians);
    }
    
    public void setModuleState (SwerveModuleState moduleState){
        velocitySpeedMotor(moduleState.speedMetersPerSecond);
        angleSteerMotor(moduleState.angle);
    }
    
    public SwerveModuleState getModuleState (){
        return new SwerveModuleState(getVelocitySpeedMotor(), getSteerMotorAngle());
    }
    
    public void setModulePosition (SwerveModulePosition swerveModulePosition){
        angleSteerMotor(swerveModulePosition.angle);
        encoder_speedMotor.setPosition(swerveModulePosition.distanceMeters);
    }
    
    public SwerveModulePosition getModulePosition (){
        return new SwerveModulePosition(getPositionSpeedMotor(), getSteerMotorAngle());
    }

    public void setIdleMode (IdleMode idleMode){
        speedMotor.setIdleMode(idleMode);
        steerMotor.setIdleMode(idleMode);
    }

    public void outputTelemetry (){
        tabDrivePositions.addDouble("SpeedMotor Velocity" + speedMotor.getDeviceId(), () -> {return getVelocitySpeedMotor();});
        tabDrivePositions.addDouble("SpeedMotor Position" + speedMotor.getDeviceId(), () -> {return getPositionSpeedMotor();});
        tabDrivePositions.addDouble("CanCoder Angle (Degrees)" + steerMotor.getDeviceId(), () -> {return getCanCoderAngle().getDegrees();});
        tabDrivePositions.addDouble("SteerMotor Angle (Degrees)" + steerMotor.getDeviceId(), () -> {return getSteerMotorAngle().getDegrees();});
    }
}
