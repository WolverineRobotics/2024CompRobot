package frc.robot.commands.AutoPositionsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakePositionCommand extends Command {
    private IntakeSubsystem m_intake; 

    public AutoIntakePositionCommand(IntakeSubsystem intakeSubsystem) {
        m_intake = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }
    @Override
    public void initialize() {System.out.println("Command Started");}

    @Override
    public void execute() { 
      m_intake.setIntakeSetpoint(-20);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
