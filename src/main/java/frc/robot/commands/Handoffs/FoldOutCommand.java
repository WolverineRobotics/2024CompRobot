package frc.robot.commands.Handoffs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.InputSystem;
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
        intake.Brake();
        intake.enable();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.disable();
        intake.Coast();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (InputSystem.AutoShutdown()) {
            return true;
        }
        else {return intake.getController().atGoal();}
    }
}
