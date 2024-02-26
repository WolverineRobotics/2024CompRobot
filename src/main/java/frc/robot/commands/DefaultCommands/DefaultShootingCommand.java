package frc.robot.commands.DefaultCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShootingCommand extends Command{

    private ShooterSubsystem m_shooter;

    public DefaultShootingCommand(ShooterSubsystem shooter_subsystem){
        m_shooter = shooter_subsystem;
        addRequirements(shooter_subsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
