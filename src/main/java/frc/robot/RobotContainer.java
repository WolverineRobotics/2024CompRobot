// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultCommands.DefaultDriveCommand;
import frc.robot.commands.DefaultCommands.DefaultShootingCommand;
import frc.robot.commands.Drive.DecelerateDriveCommand;
import frc.robot.commands.Groups.PosessGamepieceCommand;
import frc.robot.commands.Groups.StartingPositionsCommand;
import frc.robot.commands.Handoffs.StandardHandoffCommand;
import frc.robot.commands.Limelight.LimelightAlignCommand;
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

  /* Limelight */
  private LimelightSubsystem m_limelight = new LimelightSubsystem();
  private Command m_limelightAlignCommand = new LimelightAlignCommand(m_limelight);

  /* Shooter */
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private Command m_shootingcommand = new DefaultShootingCommand(m_shooter);

  /* Intake */
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private IntakeSubsystem m_intakecommand = new IntakeSubsystem();
  
  /* Controllers */ 
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public static RobotContainer instance;

  // private static final SequentialCommandGroup postPosessionCommandGroup;
  // private static final SequentialCommandGroup acquiredGamepieceCommandGroup = new SequentialCommandGroup(
  //   new );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivecommand = new DefaultDriveCommand(m_drive);
    m_shootingcommand = new DefaultShootingCommand(m_shooter);
    m_limelightAlignCommand = new LimelightAlignCommand(m_limelight);
    
    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_drivecommand);
    CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);
    CommandScheduler.getInstance().setDefaultCommand(m_limelight, m_limelightAlignCommand);

    instance = this;
    // configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().whileTrue(new DecelerateDriveCommand(m_drive));
    m_driverController.rightBumper().whileTrue(new LimelightAlignCommand(m_limelight));
  }

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
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_drive);
    return null;
  }
  
}

