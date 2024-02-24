// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DecelerateDriveCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShootingCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  /* Drive */
  private DriveSubsystem m_drive = new DriveSubsystem();
  private Command m_drivecommand = new DefaultDriveCommand(m_drive);

  /* Limelight */
  // private LimelightSubsystem m_limelight = new LimelightSubsystem();
  // private Command m_limelightCommand = new LimelightAlignCommand(m_drive, m_limelight);

  /* Shooter */
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private Command m_shootingcommand = new DefaultShootingCommand(m_shooter);
  
  /* Controllers */ 
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivecommand = new DefaultDriveCommand(m_drive);
    m_shootingcommand = new DefaultShootingCommand(m_shooter);

    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_drivecommand);
    CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);

    // CameraServer.startAutomaticCapture();
    // configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().whileTrue(new DecelerateDriveCommand(m_drive));
    // joeMama.driveDecelerate.whileTrue(new DecelerateDriveCommand(m_drive));
  }

  public DriveSubsystem VroomVroom(){
    return m_drive;
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_drive);
    return null;
  }
  
}

