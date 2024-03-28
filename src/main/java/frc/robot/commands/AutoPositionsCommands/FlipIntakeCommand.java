package frc.robot.commands.AutoPositionsCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Robot;

public class FlipIntakeCommand extends Command {

    private final IntakeSubsystem m_Intake;

    private double intakeSetpoint;
    private double intakeError;
    private double errorTolerance = 1;

    public FlipIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeAngle){
        m_Intake = intakeSubsystem;
        intakeSetpoint = intakeAngle;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double kP_Intake = 0.05;
        double currentPos = m_Intake.pivotCanEncoder.getPosition();
        double intakeError = (intakeSetpoint - currentPos);

        m_Intake.pivotMotor.set(intakeError * kP_Intake);

        if (intakeError > 0){
            m_Intake.pivotMotor.set(-intakeError*kP_Intake);
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
        if(Math.abs(intakeError) <= errorTolerance){
            return true;
        } else {
            return false;
        }
    }
}
