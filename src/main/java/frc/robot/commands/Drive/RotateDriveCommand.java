package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateDriveCommand extends Command{
    private final DriveSubsystem m_drive; 

    public RotateDriveCommand(DriveSubsystem subsystem) {
        m_drive = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

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
        return false;
    }
}