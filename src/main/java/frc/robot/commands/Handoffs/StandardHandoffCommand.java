package frc.robot.commands.Handoffs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StandardHandoffCommand extends Command{
        private ShooterSubsystem shooter;
        private IntakeSubsystem intake;

        private int stage;

    public StandardHandoffCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
        stage = 1;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setGoal(Constants.Positional.kShooterNonConflictPosition);
        shooter.enable();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(shooter.getController().atGoal() && stage == 1){
            intake.setGoal(Constants.Positional.kIntakeDefaultFeedPosition); 
            intake.enable();
            stage = 2;
        }
        else if (intake.getController().atGoal() && stage == 2){
            shooter.setGoal(Constants.Positional.kShooterDefaultFeedPosition);
            stage = 3;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (InputSystem.AutoShutdown()) {
        //     return true;
        // }
        // else {return (shooter.getController().atGoal() && intake.getController().atGoal() && stage == 3);}
        return false;
    }
}
