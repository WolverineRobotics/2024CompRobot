package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;

public class DefaultClimbCommand extends Command{
    Robot.ClimbSubsystem
    @Override
    public void execute() {
        if (Input.driveController.getLeftTriggerAxis() > 0.1){leftClimb.set(0.3);}
        else if(Input.driveController.getLeftBumper()){ leftClimb.set(-0.3); }
        else{ leftClimb.set(0); }
        
        if (Input.driveController.getRightTriggerAxis() > 0.1){ rightClimb.set(-0.3); }
        else if(Input.driveController.getRightBumper()){rightClimb.set(0.3); }
        else{rightClimb.set(0); }

    }
    
}
