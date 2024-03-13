package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PostShotCommand extends Command{
        private ShooterSubsystem shooter;
        private IntakeSubsystem intake;

    // public PostShotCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    public PostShotCommand(IntakeSubsystem intakeSubsystem) {
        // shooter = shooterSubsystem;
        intake = intakeSubsystem;
        // addRequirements(shooterSubsystem, intakeSubsystem);
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // shooter.setGoal(Constants.Positional.kIntakeSubwooferHandoffPosition); 
        intake.setGoal(Constants.Positional.kShooterMinPosition); 

        // shooter.enable();
        intake.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return shooter.getController().atGoal() && intake.getController().atGoal();
        return intake.getController().atGoal();
    }
}
