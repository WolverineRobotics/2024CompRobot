package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.commands.AutoPositionsCommands.AutoIntakeCommand;
import frc.robot.commands.AutoPositionsCommands.FlipIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{

    private IntakeSubsystem m_intake;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem){
        m_intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Intake the notes
        if(Input.fireInTheHole() > 0){

            if(!m_intake.intakeLimitSwitch.get()){ 
                m_intake.setIntakeSpeed(0);

            } else {
                m_intake.setIntakeSpeed(1);
            }

        }

        // Shooting out of intake
        else if(Input.AmpScore()){
            m_intake.setIntakeSpeed(-0.9);
        }

        else{
            m_intake.setIntakeSpeed(0);
        }
        
        // Manual intake pivot control
        m_intake.pivotMotor.set(Input.Operator().getRightY() * 0.3);

        // Check if intake going into robot
        if (m_intake.pivotCanEncoder.getPosition() > -3 && Input.Operator().getRightY() > 0){
            m_intake.pivotMotor.set(0);
        }

        // Check if intake going into floor
        if (m_intake.pivotCanEncoder.getPosition() < -39 && Input.Operator().getRightY() < 0) { 
            m_intake.pivotMotor.set(0);
        } 

        // Reset the intake once limit switch has been hit
        // if (m_intake.intakeLimitSwitch2.get()){
        //     m_intake.ResetPivotEncoder();
        // }

        // AUTO INTAKE COMMAND
        // if(Input.Operator().getAButton()){
        //     CommandScheduler.getInstance().schedule(new AutoIntakeCommand(m_intake));
        // }
        
        // AUTO FLIP OUT
        // if(Input.Operator().getXButton()){
        //     CommandScheduler.getInstance().schedule(new FlipIntakeCommand(m_intake, Constants.Positional.kIntakeIntakingPosition));
        // }
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