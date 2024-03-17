package frc.robot.commands.Handoffs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.commands.AutoPositionsCommands.AutoIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class FlipIntakeCommand extends Command {
    private final IntakeSubsystem m_Intake;
    private double intakeSetpoint;

    public FlipIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeAngle){
        m_Intake = intakeSubsystem;
        intakeSetpoint = intakeAngle;
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double kP_Intake = 1;
        double intakeError = (intakeSetpoint - m_Intake.pivotCanEncoder.getPosition());

        if(intakeError > 0){
            m_Intake.pivotMotor.set(0);

        } else if (intakeError < 0) {
            m_Intake.pivotMotor.set(intakeError * kP_Intake);

        }

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
