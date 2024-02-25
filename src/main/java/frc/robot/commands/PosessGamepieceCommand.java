package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PosessGamepieceCommand extends Command{
        private ShooterSubsystem shooter;
        private IntakeSubsystem intake;

        private int stage;
        private final double posessionMotorSpeedConstant = 0.2;

    public PosessGamepieceCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
        stage = 1;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!shooter.shooterLimitSwitch.get()){
            shooter.setPosessionSpeed(posessionMotorSpeedConstant);
            shooter.setShooterSpeed(posessionMotorSpeedConstant * 0.5);
            intake.setIntakeSpeed(-posessionMotorSpeedConstant * 0.5);
        }
        else{
            shooter.setPosessionSpeed(-posessionMotorSpeedConstant * 0.5);
            shooter.setShooterSpeed(-posessionMotorSpeedConstant);
            intake.setIntakeSpeed(0);
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        shooter.setPosessionSpeed(0);
        shooter.setShooterSpeed(0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (shooter.posessionLimitSwitch.get() && !shooter.shooterLimitSwitch.get());
    }
}