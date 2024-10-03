package frc.robot.commands.AutoPositionsCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command{

    private IntakeSubsystem mIntake;
    public AutoIntakeCommand(IntakeSubsystem intakeSubsystem /* Debouncer mDebouncer */){
        mIntake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!mIntake.intakeLimitSwitch.get()) {
            Input.driveController.setRumble(RumbleType.kBothRumble, 0);
            Input.opController.setRumble(RumbleType.kBothRumble, 0);
            mIntake.intakeMotor.set(0);

        } else {
            Input.driveController.setRumble(RumbleType.kBothRumble, 1);
            Input.opController.setRumble(RumbleType.kBothRumble, 1);
            mIntake.intakeMotor.set(1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.intakeMotor.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return mDebouncer.calculate(!mIntake.intakeLimitSwitch.get());
        return !mIntake.intakeLimitSwitch.get();
    }

}