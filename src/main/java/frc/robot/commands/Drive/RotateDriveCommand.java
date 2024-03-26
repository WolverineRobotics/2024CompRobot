package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class RotateDriveCommand extends Command{
    private final DriveSubsystem m_drive; 
    private int target;

    private ProfiledPIDController pid = new ProfiledPIDController(0.02, 0, 0.0, new Constraints(350, 200));
    // private PIDController pid = new PIDController(0.015, 0, 0);

    public RotateDriveCommand(DriveSubsystem drive, int targetRotation) {
        m_drive = drive;

        target = targetRotation;
        addRequirements(drive);
        pid.setTolerance(10);
        SmartDashboard.putNumber("PID SETPOINT", pid.getGoal().position);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        pid.reset(m_drive.GetHeading());
        pid.setGoal(target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        m_drive.AutoDrive(-pid.calculate(m_drive.GetHeading()), 0);
        SmartDashboard.putNumber("ERROR", pid.getPositionError());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.AutoDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
            // return false;

        return pid.atGoal();
    }
}