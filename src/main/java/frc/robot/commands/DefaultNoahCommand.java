package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.InputSystem;
import frc.robot.subsystems.NoahDriveSubsystem;

public class DefaultNoahCommand extends Command{
    private NoahDriveSubsystem test;


    public DefaultNoahCommand(NoahDriveSubsystem noah) {
        test = noah; 
        addRequirements(test);
        
    }

    @Override
    public void execute() {
       
        test.getDriveTrain().arcadeDrive(InputSystem.DriveSpeed(), InputSystem.DriveRot());   
    }
    @Override
    public void end(boolean interrupted) {
       
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
