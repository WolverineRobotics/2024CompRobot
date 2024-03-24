package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateDriveCommand extends Command{
    private final DriveSubsystem m_drive; 


    private PIDController pid = new PIDController(0.03, 0, 0);

    public RotateDriveCommand(DriveSubsystem drive, int targetRotation) {
        m_drive = drive;

        pid.setSetpoint(targetRotation);

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        m_drive.AutoDrive(
            0,
            pid.calculate(m_drive.GetHeading())
            );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.AutoDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return pid.atSetpoint();
    }
}