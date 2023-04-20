
package frc.robot;

import frc.robot.autos.Auto1_FirstTrajectory;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private CommandXboxController Control0 = new CommandXboxController(0);
  private final SwerveDrive m_SwerveDrive = new SwerveDrive();
  private final AutonomousBuilder autonomousBuilder = new AutonomousBuilder(m_SwerveDrive);
  SendableChooser<Command> m_chooser_auto = new SendableChooser<>();
  private final Auto1_FirstTrajectory auto1 = new Auto1_FirstTrajectory();

  public RobotContainer() {
    m_chooser_auto.setDefaultOption("NoAutoSelected", null);
    m_chooser_auto.addOption("2Scores + 1Piece + ChargeStation", autonomousBuilder.createCommand(auto1));
    m_chooser_auto.addOption("2Scores + ChargeStation", null);
    m_chooser_auto.addOption("2Scores + Parking", null);
    m_chooser_auto.addOption("3Scores", null);
    SmartDashboard.putData("Auto", m_chooser_auto);
    configureBindings();
  }

  private void configureBindings() {
    Trigger changeIdleMode = new Trigger(RobotState::isEnabled);
    changeIdleMode.toggleOnTrue(
      new InstantCommand(
        () -> {m_SwerveDrive.setBrakeMode();}
      )
    ).toggleOnFalse(
      new InstantCommand(
        () -> {m_SwerveDrive.setCoastMode();}
      )
    );
    Trigger modeChanged = new Trigger(GeneralMode.getInstance()::haveChanged);
    modeChanged.toggleOnTrue(
      new ParallelCommandGroup(
        //Ejemplo de como usar el cambio de estado para mecanismos
        //m_SwerveDrive.getCurrentCommand()
      )
    );
    Control0.a().onTrue(GeneralMode.getInstance().toggleMode());
  }

  public Command getAutonomousCommand() {
    return m_chooser_auto.getSelected();
  }
}
