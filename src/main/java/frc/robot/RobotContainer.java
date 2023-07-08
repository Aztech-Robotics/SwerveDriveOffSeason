
package frc.robot;

import frc.robot.actions.FieldOrientedDrive;
import frc.robot.autos.AutoSample;
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
  private final SwerveDrive m_SwerveDrive = new SwerveDrive();
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(
    m_SwerveDrive, 
    () -> {return Control0.getLeftY();}, 
    () -> {return Control0.getLeftX();}, 
    () -> {return Control0.getRightX();}
  );
  private AutonomousBuilder autonomousBuilder = new AutonomousBuilder(m_SwerveDrive);
  private AutoSample autoSample = new AutoSample();
  private SendableChooser<Command> m_chooser_auto = new SendableChooser<>();

  public RobotContainer() {
    m_SwerveDrive.setDefaultCommand(m_FieldOrientedDrive);
    m_chooser_auto.setDefaultOption("NoAuto", null);
    m_chooser_auto.addOption("AutoSample", autonomousBuilder.createCommand(autoSample));
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
    return m_chooser_auto.getSelected();
  }
}
