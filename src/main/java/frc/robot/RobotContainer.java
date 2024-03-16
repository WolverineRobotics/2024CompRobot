// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultCommands.DefaultDriveCommand;
import frc.robot.commands.DefaultCommands.DefaultShootingCommand;
import frc.robot.commands.Drive.DecelerateDriveCommand;
import frc.robot.commands.Drive.RotateDriveCommand;
import frc.robot.commands.Groups.PosessGamepieceCommand;
import frc.robot.commands.Groups.StartingPositionsCommand;
import frc.robot.commands.Handoffs.FoldBackCommand;
import frc.robot.commands.Handoffs.FoldOutCommand;
import frc.robot.commands.Handoffs.StandardHandoffCommand;
import frc.robot.commands.Limelight.LimelightAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.NoahDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  /* Drive */
  private DriveSubsystem m_Drive = new DriveSubsystem();
  private Command m_Drivecommand = new DefaultDriveCommand(m_Drive);

  /* Noah */
  //private NoahDriveSubsystem m_Noah = new NoahDriveSubsystem();

  /* Limelight */
  private LimelightSubsystem m_Limelight = new LimelightSubsystem();
  private Command m_LimelightAlignCommand = new LimelightAlignCommand(m_Limelight);

  /* Shooter */
  // private ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private Command m_shootingcommand = new DefaultShootingCommand(m_shooter);

  /* Intake */
  private IntakeSubsystem m_Intake = new IntakeSubsystem();
  // private IntakeSubsystem m_intakecommand = new IntakeSubsystem();
  
  /* Controllers */ 
  public static CommandXboxController m_DriverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static CommandXboxController m_OperatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public static RobotContainer instance;

  // private static final SequentialCommandGroup postPosessionCommandGroup;
  // private static final SequentialCommandGroup acquiredGamepieceCommandGroup = new SequentialCommandGroup(
  //   new );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_Drivecommand = new DefaultDriveCommand(m_Drive);
    // m_shootingcommand = new DefaultShootingCommand(m_shooter);
    m_LimelightAlignCommand = new LimelightAlignCommand(m_Limelight);
    
    CommandScheduler.getInstance().setDefaultCommand(m_Drive, m_Drivecommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);
    CommandScheduler.getInstance().setDefaultCommand(m_Limelight, m_LimelightAlignCommand);

    instance = this;
    // configureBindings();
  }

  private void configureBindings() {
    m_DriverController.leftBumper().whileTrue(new DecelerateDriveCommand(m_Drive));
    m_DriverController.rightBumper().whileTrue(new LimelightAlignCommand(m_Limelight));
  }

  public DriveSubsystem VroomVroom(){
    return m_Drive;
  }

  /*public NoahDriveSubsystem drive(){
    return m_Noah;
  }*/
  
  public void PostPosessionRoutine(){
    // CommandScheduler.getInstance().schedule(new StartingPositionsCommand(m_intake));
    CommandScheduler.getInstance().schedule(new FoldBackCommand(m_Intake));
  }

  public void StartIntaking(){
    CommandScheduler.getInstance().schedule(
      new FoldOutCommand(m_Intake)
    );
  }

  public void AcquiredGamepiece(){
    // Called when the gamepiece is acquired via intake
    // Schedules the handoff by default

    CommandScheduler.getInstance().schedule(
      new FoldBackCommand(m_Intake)
    );

    // CommandScheduler.getInstance().schedule(
    //   new StandardHandoffCommand(m_intake).andThen(
    //     new PosessGamepieceCommand(m_intake) ) );
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_drive);
    return null;
  }
  
}

