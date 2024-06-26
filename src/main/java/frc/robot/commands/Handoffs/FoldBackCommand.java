package frc.robot.commands.Handoffs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FoldBackCommand extends Command{
        
        private IntakeSubsystem intake;

    public FoldBackCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setGoal(Constants.Positional.kIntakeDefaultFeedPosition); 
        Robot.pivotIsMoving = true;
        Robot.isFoldedBack = false;
        // intake.enable();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setPivotSpeed(0.5);
        intake.setIntakeSpeed(0);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // intake.disable();
        intake.setPivotSpeed(0);
        if(!interrupted) {Robot.pivotIsMoving = false;}

        Robot.ResetFoldedBackTimer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Input.AutoShutdown()) {
            return true;
        }
        // else {return intake.getController().atGoal();}
        else {return intake.getEncoderPosition() >= 0 ;}

        // return false;
    }
}
