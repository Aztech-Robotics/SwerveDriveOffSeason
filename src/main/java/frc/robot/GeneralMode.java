package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.GeneralModeEnum;

public class GeneralMode {
    private static GeneralMode generalMode = null;
    private static GeneralModeEnum activeMode;
    private boolean notifier = false;

    private GeneralMode (){}

    public static GeneralMode getInstance (){
        if (generalMode == null){
            generalMode = new GeneralMode();
            activeMode = GeneralModeEnum.Cone;
        }
        return generalMode;
    }
    
    public GeneralModeEnum getMode (){
        return activeMode; 
    }
    public void setMode (GeneralModeEnum mode){
        notifier = true;
        activeMode = mode;
    }

    public InstantCommand toggleMode (){
        notifier = true;
        InstantCommand command = new InstantCommand(
            () -> {
                if (activeMode == GeneralModeEnum.Cone){
                    activeMode = GeneralModeEnum.Cube;
                }
                else {
                    activeMode = GeneralModeEnum.Cone;
                }
            }
        );
        return command;
    }

    public boolean haveChanged (){
        if (notifier){
            notifier = false;
            return true;
        }
        else {
            return false;
        }
    }
}
