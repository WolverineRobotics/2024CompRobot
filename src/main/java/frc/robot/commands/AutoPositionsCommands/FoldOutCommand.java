package frc.robot.commands.AutoPositionsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FoldOutCommand extends Command{ 
    private IntakeSubsystem intake;

    public FoldOutCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setGoal(Constants.Positional.kIntakeIntakingPosition); 
        Robot.pivotIsMoving= true;
        Robot.isFoldedBack = false;
        // intake.enable();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setPivotSpeed(-0.5);
        intake.setIntakeSpeed(-0.6);

    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.disable();
        if(!interrupted) {Robot.pivotIsMoving = false;}
        intake.setPivotSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Input.AutoShutdown()) {
            return true;
        }
        // else {return intake.getController().atGoal();}
        else {return intake.getEncoderPosition() <= -38 ;}
        // return false;
    }
}
