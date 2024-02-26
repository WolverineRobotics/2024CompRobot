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
import frc.robot.commands.PosessGamepieceCommand;
import frc.robot.commands.ShooterLimelightCommand;
import frc.robot.commands.StartingPositionsCommand;
import frc.robot.commands.Handoffs.StandardHandoffCommand;
import frc.robot.commands.Handoffs.SubwooferHandoffCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  /* Drive */
  private DriveSubsystem m_drive = new DriveSubsystem();
  private Command m_drivecommand = new DefaultDriveCommand(m_drive);
  private LimelightSubsystem m_limelight = new LimelightSubsystem();
  
  /* Limelight */
  // private Command m_limelightCommand = new LimelightAlignCommand(m_drive, m_limelight);
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  
  /* Shooter */
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private Command m_shootingcommand = new DefaultShootingCommand(m_shooter);
  
  private ShooterLimelightCommand m_shooter_aim_command = new ShooterLimelightCommand(m_shooter, m_limelight);
  
  public static RobotContainer instance;

  private final SequentialCommandGroup auto_shoot_sequence = new SequentialCommandGroup(
    new SubwooferHandoffCommand(m_shooter, m_intake),
    new PosessGamepieceCommand(m_shooter, m_intake),
    new ShooterLimelightCommand(m_shooter, m_limelight)
    // auto shoot command
    );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivecommand = new DefaultDriveCommand(m_drive);
    m_shootingcommand = new DefaultShootingCommand(m_shooter);

    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_drivecommand);
    CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);

    instance = this;
    // CameraServer.startAutomaticCapture();
    // configureBindings();
  }

  // private void configureBindings() {
  //   m_driverController.leftBumper().whileTrue(new DecelerateDriveCommand(m_drive));
  //   // joeMama.driveDecelerate.whileTrue(new DecelerateDriveCommand(m_drive));
  // }

  public DriveSubsystem VroomVroom(){
    return m_drive;
  }

  public void PostPosessionRoutine(){
    CommandScheduler.getInstance().schedule(new StartingPositionsCommand(m_shooter, m_intake));
  }

  public void AcquiredGamepiece(){
    // Called when the gamepiece is acquired via intake
    // Schedules the handoff by default
    CommandScheduler.getInstance().schedule(
      new StandardHandoffCommand(m_shooter, m_intake).andThen(
        new PosessGamepieceCommand(m_shooter, m_intake) ) );
  }
  
  public void ShooterAcquiredGamepiece(){
    // Called when the gamepiece is acquired via shooter
    // Schedules the handoff by default
    CommandScheduler.getInstance().schedule(
        new PosessGamepieceCommand(m_shooter, m_intake) );
  }
  
  public void AutoAlign(boolean align){
      if (align) {
        m_shooter_aim_command.SetContinuous(true);
        CommandScheduler.getInstance().schedule(m_shooter_aim_command);
      }
      else{
        m_shooter_aim_command.cancel();
        m_shooter_aim_command = new ShooterLimelightCommand(m_shooter, m_limelight);
      }
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_drive);
    return null;
  }
  
}

