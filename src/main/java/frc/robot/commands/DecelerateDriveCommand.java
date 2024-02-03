// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Input;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DecelerateDriveCommand extends Command {
  private final DriveSubsystem m_drive;

  public DecelerateDriveCommand(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // X, Y Movement + Bumper
    // double throttle = Input.getHorizontal();
    // double turn = Input.getVertical(); 

    // For Deceleration
    double yspeedReduction = Constants.Y_SPEED_REDUCTION;
    double xspeedReduction = Constants.X_SPEEDREDUCTION;
    arcadeDrive(xspeedReduction, yspeedReduction);
  }

  public void arcadeDrive(double xspeedReduction, double yspeedReduction){
    DriveSubsystem.driveTrain.arcadeDrive(Input.getHorizontal() * xspeedReduction, Input.getVertical() * yspeedReduction);
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
