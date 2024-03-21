package frc.robot.commands.Handoffs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwooferHandoffCommand extends Command{
        private ShooterSubsystem shooter;
        private IntakeSubsystem intake;

    public SubwooferHandoffCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setGoal(Constants.Positional.kShooterSubwooferShotPosition); 
        //intake.setGoal(Constants.Positional.kIntakeSubwooferHandoffPosition); 

        shooter.enable();
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
        // if (InputSystem.AutoShutdown()) {
        //     return true;
        // }
        // else {return shooter.getController().atGoal() && intake.getController().atGoal();}
        return false;
    }
}
