
package frc.robot;

import frc.robot.actions.CalibrateModule;
import frc.robot.actions.CalibrateModuleAngle;
import frc.robot.actions.FieldOrientedDrive;
import frc.robot.autos.FullAuto;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private CommandXboxController Control0 = new CommandXboxController(0);
  private CommandXboxController Control1 = new CommandXboxController(1);
  private final SwerveDrive m_SwerveDrive = new SwerveDrive();
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(
    m_SwerveDrive, 
    () -> {return -Control0.getLeftY();}, 
    () -> {return Control0.getLeftX();}, 
    () -> {return Control0.getRightX();}
  );
  //private final AutonomousBuilder autonomousBuilder = new AutonomousBuilder(m_SwerveDrive);
  //SendableChooser<Command> m_chooser_auto = new SendableChooser<>();
  //private final FullAuto auto1 = new FullAuto();

  public RobotContainer() {
    //m_chooser_auto.setDefaultOption("NoAutoSelected", null);
    //m_chooser_auto.addOption("2Scores + 1Piece + ChargeStation", autonomousBuilder.createCommand(auto1));
    //SmartDashboard.putData("Auto", m_chooser_auto);
    ShuffleboardTab tabRobot = Shuffleboard.getTab("RobotData");
    tabRobot.addBoolean("Robot Enabled", RobotState::isEnabled);
    m_SwerveDrive.setDefaultCommand(m_FieldOrientedDrive);
    configureBindings();
  }

  private void configureBindings() {
    Trigger idleMode = new Trigger(RobotState::isEnabled);
    idleMode.onTrue(
      new InstantCommand(
        () -> {m_SwerveDrive.setBrakeMode();}
      )
    );
    idleMode.onFalse(
      new InstantCommand(
        () -> {m_SwerveDrive.setCoastMode();}
      )
    );
    /*
    Trigger modeChanged = new Trigger(GeneralMode.getInstance()::haveChanged);
    modeChanged.toggleOnTrue(
      new ParallelCommandGroup(
        //Ejemplo de como usar el cambio de estado para mecanismos
        //m_SwerveDrive.getCurrentCommand()
      )
    );
    */

  }

  public Command getAutonomousCommand() {
    return null; 
    //return m_chooser_auto.getSelected();
  }
}
