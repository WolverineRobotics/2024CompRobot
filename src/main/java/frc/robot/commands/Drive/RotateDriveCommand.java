package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.InputSystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateDriveCommand extends Command{
    private final DriveSubsystem rotate; 

    public RotateDriveCommand(DriveSubsystem subsystem) {
        rotate = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Check For Possible Inputs
        if (InputSystem.FaceForward()) {
            rotate.getController().setGoal(Constants.Positional.kFaceForward);
        }

        if (InputSystem.FaceLeft()) {
            rotate.getController().setGoal(Constants.Positional.kFaceLeft);
        }

        if (InputSystem.FaceRight()) {
            rotate.getController().setGoal(Constants.Positional.kFaceRight);
        }

        if (InputSystem.FaceDriver()) {
            rotate.getController().setGoal(Constants.Positional.kFaceDriver);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rotate.getController().atGoal();
    }
}