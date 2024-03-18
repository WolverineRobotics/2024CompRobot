package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimbCommand extends Command{
    private final ClimbSubsystem mClimb;

    public DefaultClimbCommand(ClimbSubsystem climb) {
        mClimb = climb;
        addRequirements(mClimb);
    }

    @Override
    public void execute() {
        if (Input.driveController.getLeftTriggerAxis() > 0.1){mClimb.leftClimb.set(0.3);}
        else if(Input.driveController.getLeftBumper()){mClimb.leftClimb.set(-0.3); }
        else{mClimb.leftClimb.set(0); }
        
        if (Input.driveController.getRightTriggerAxis() > 0.1){mClimb.rightClimb.set(-0.3); }
        else if(Input.driveController.getRightBumper()){mClimb.rightClimb.set(0.3); }
        else{mClimb.rightClimb.set(0); }

    }
    
}
