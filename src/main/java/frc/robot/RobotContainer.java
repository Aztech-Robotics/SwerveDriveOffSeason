
package frc.robot;

import frc.robot.Constants.SwerveMode;
import frc.robot.actions.FieldOrientedDrive;
import frc.robot.autos.AutoPath1;
import frc.robot.autos.AutoPath2;
import frc.robot.autos.AutoPath3;
import frc.robot.autos.AutoPath4;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private CommandXboxController Control0 = new CommandXboxController(0);
  private SwerveDrive m_SwerveDrive = new SwerveDrive();
  private FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(
    m_SwerveDrive, 
    () -> {return Control0.getLeftY();}, 
    () -> {return Control0.getLeftX();}, 
    () -> {return Control0.getRightX();}
  );
  private AutonomousBuilder autonomousBuilder = new AutonomousBuilder(m_SwerveDrive);
  private SendableChooser<Integer> m_chooser_auto = new SendableChooser<>();
  private AutoPath1 autoPath1 = new AutoPath1();
  private AutoPath2 autoPath2 = new AutoPath2();
  private AutoPath3 autoPath3 = new AutoPath3();
  private AutoPath4 autoPath4 = new AutoPath4();

  public RobotContainer() {
    m_SwerveDrive.setDefaultCommand(m_FieldOrientedDrive);
    m_chooser_auto.setDefaultOption("NoAuto", 0);
    m_chooser_auto.addOption("AutoPath1", 1); 
    m_chooser_auto.addOption("FullAuto", 2); 
    Shuffleboard.getTab("SwerveData").add("AutoSelected", m_chooser_auto).withSize(2, 1).withPosition(8, 2);
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
    int auto = m_chooser_auto.getSelected();
    Command autoCommand;
    switch (auto){
      case 1:
      autoCommand = autonomousBuilder.createCommand(autoPath1);
      break;
      case 2:
      autoCommand = new SequentialCommandGroup(
      autonomousBuilder.createCommand(autoPath1),
      autonomousBuilder.createCommand(autoPath2),
      autonomousBuilder.createCommand(autoPath3),
      autonomousBuilder.createCommand(autoPath4)
    );
      break;
      default:
      autoCommand = null;
      break;
    }
    return autoCommand;
  }
}
