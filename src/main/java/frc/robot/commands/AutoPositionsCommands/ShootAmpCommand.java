package frc.robot.commands.AutoPositionsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Robot;

public class ShootAmpCommand extends Command{

    private IntakeSubsystem mIntake;
    private int _milliseconds;

    public ShootAmpCommand(IntakeSubsystem intakeSubsystem, int milliseconds){
        mIntake = intakeSubsystem;
        _milliseconds = milliseconds;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("AAAAAA SHOOTCOMMAND STARTED EEEEEEEEEEEE");
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {        
        mIntake.intakeMotor.set(-1);
        _milliseconds -= 20;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.intakeMotor.set(0);
        System.out.println("AAAAAA SHOOTCOMMAND FINISHED EEEEEEEEEEEE");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _milliseconds <= 0;
    }

}