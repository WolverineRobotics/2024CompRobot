package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.InputSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartingPositionsCommand extends Command{
        private ShooterSubsystem shooter;
        private IntakeSubsystem intake;

    // public StartingPositionsCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    public StartingPositionsCommand(IntakeSubsystem intakeSubsystem) {
        // shooter = shooterSubsystem;
        intake = intakeSubsystem;

        // addRequirements(shooterSubsystem, intakeSubsystem);
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // shooter.setGoal(Constants.Positional.kShooterSubwooferShotPosition); 
        intake.setGoal(Constants.Positional.kIntakeSubwooferHandoffPosition); 

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
        if (InputSystem.AutoShutdown()) {
            return true;
        }
        else {return intake.getController().atGoal();}
    }
}
