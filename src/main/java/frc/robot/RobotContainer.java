// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultCommands.DefaultClimbCommand;
import frc.robot.commands.DefaultCommands.DefaultDriveCommand;
import frc.robot.commands.DefaultCommands.DefaultIntakeCommand;
import frc.robot.commands.DefaultCommands.DefaultShootingCommand;
import frc.robot.commands.Drive.DecelerateDriveCommand;
import frc.robot.commands.Drive.ForwardDrive;
import frc.robot.commands.Drive.RotateDriveCommand;
import frc.robot.commands.Groups.PosessGamepieceCommand;
import frc.robot.commands.Groups.StartingPositionsCommand;
import frc.robot.commands.Handoffs.FoldBackCommand;
import frc.robot.commands.Handoffs.FoldOutCommand;
import frc.robot.commands.Handoffs.StandardHandoffCommand;
import frc.robot.commands.Limelight.LimelightAlignCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private DefaultIntakeCommand m_IntakeCommand = new DefaultIntakeCommand(m_Intake);
  
  private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private DefaultClimbCommand m_ClimbCommand = new DefaultClimbCommand(m_ClimbSubsystem);
  
  private PIDController left_pid = new PIDController(0.01, 0, 0);
  private PIDController right_pid = new PIDController(0.01, 0, 0);

  public static RobotContainer instance;

  private final SimpleMotorFeedforward trajFeedforward = 
    new SimpleMotorFeedforward(0.25, 1);

  private final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    trajFeedforward, Constants.kDriveKinematics, 4);
    
  private final TrajectoryConfig trajConfig = new TrajectoryConfig(2, 2)
    .setKinematics(Constants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

  
  private final RamseteController rController = new RamseteController();
  private Trajectory test = 
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
       List.of( new Translation2d(0.5, 0) ) , new Pose2d(1, 0,
        new Rotation2d(0)),
         trajConfig);

  private RamseteCommand testTrajectory = new RamseteCommand(
    test,
    m_Drive::GetPose,
    rController, 
    trajFeedforward,
    Constants.kDriveKinematics, 
    m_Drive::GetWheelSpeeds,
    left_pid,
    right_pid,
    m_Drive::SetDriveVoltages,
    m_Drive );

  // private final TrajectoryConstraint trajConstraint = 

  // private static final SequentialCommandGroup postPosessionCommandGroup;
  // private static final SequentialCommandGroup acquiredGamepieceCommandGroup = new SequentialCommandGroup(
  //   new );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_Drivecommand = new DefaultDriveCommand(m_Drive);
    m_LimelightAlignCommand = new LimelightAlignCommand(m_Limelight);
    // m_shootingcommand = new DefaultShootingCommand(m_shooter);
    
    CommandScheduler.getInstance().setDefaultCommand(m_Drive, m_Drivecommand);
    CommandScheduler.getInstance().setDefaultCommand(m_Intake, m_IntakeCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_Limelight, m_LimelightAlignCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_ClimbSubsystem, m_ClimbCommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);

    instance = this;
    // configureBindings();

    SmartDashboard.putNumber("left_error", left_pid.getPositionError());
    SmartDashboard.putNumber("right_error", right_pid.getPositionError());

  }

  public DriveSubsystem VroomVroom(){
    return m_Drive;
  }

  public IntakeSubsystem getIntakeSubsystem(){
    return m_Intake;
  }

  public ClimbSubsystem getClimbSubsystem(){
    return m_ClimbSubsystem;
  }

  public LimelightSubsystem getLimelightSubsystem(){
    return m_Limelight;
  }

  private void configureBindings() {
    // m_DriverController.leftBumper().whileTrue(new DecelerateDriveCommand(m_Drive));
    // m_DriverController.rightBumper().whileTrue(new LimelightAlignCommand(m_Limelight));
  }
  
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
  }
  
  public Command getAutonomousCommand() {
    return testTrajectory;
  }
  
}

