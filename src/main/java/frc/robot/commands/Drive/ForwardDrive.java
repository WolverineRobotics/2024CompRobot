// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ForwardDrive extends Command {
  private final DriveSubsystem fDrive;
  
  private final ProfiledPIDController pid = 
    new ProfiledPIDController(0.01, 0, 0, 
    new Constraints(250, 100));

  public double initLeftEncoder;
  public double initRightEncoder;

  public double currentLeftEncoderValue;
  public double currentRightEncoderValue;
  public double newEncoderAverage;

  public ForwardDrive(DriveSubsystem driveSubsystem, double setpoint) {
    fDrive = driveSubsystem;
    pid.setGoal(setpoint);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initLeftEncoder = fDrive.getLeftEncoderPosition();
    initRightEncoder = fDrive.getRightEncoderPosition();
    pid.reset(pid.calculate(0));
    pid.setTolerance(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate Difference Between Current & Previous Encoder Value
    currentLeftEncoderValue = fDrive.getLeftEncoderPosition() - initLeftEncoder;
    currentRightEncoderValue = fDrive.getRightEncoderPosition() - initRightEncoder;
    newEncoderAverage = (currentLeftEncoderValue+currentRightEncoderValue)/2;
  
    double driveSpeed = pid.calculate(newEncoderAverage);
    fDrive.AutoDrive(0,driveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fDrive.AutoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atGoal();
  }
}
