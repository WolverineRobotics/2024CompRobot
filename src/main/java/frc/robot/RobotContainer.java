// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPositionsCommands.ShootAmpCommand;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  /* Drive */
  private DriveSubsystem m_Drive = new DriveSubsystem();
  private Command m_Drivecommand = new DefaultDriveCommand(m_Drive);

  //private DriveSubsystem limelightDrive = new DriveSubsystem();

  /* Noah */
  //private NoahDriveSubsystem m_Noah = new NoahDriveSubsystem();

  /* Limelight */
  private LimelightSubsystem m_Limelight = new LimelightSubsystem();
  // private Command m_LimelightAlignCommand = new LimelightAlignCommand(m_Limelight);

  /* Shooter */
  // private ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private Command m_shootingcommand = new DefaultShootingCommand(m_shooter);

  /* Intake */
  private IntakeSubsystem m_Intake = new IntakeSubsystem();
  private DefaultIntakeCommand m_IntakeCommand = new DefaultIntakeCommand(m_Intake);

  private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private DefaultClimbCommand m_ClimbCommand = new DefaultClimbCommand(m_ClimbSubsystem);
  private Debouncer limitDebouncer = new Debouncer(0.0005);

  public static RobotContainer instance;

  // private static final SequentialCommandGroup postPosessionCommandGroup;
  // private static final SequentialCommandGroup acquiredGamepieceCommandGroup = new SequentialCommandGroup(
  //   new );

  private final SequentialCommandGroup rotateTest = new SequentialCommandGroup(
    new ForwardDrive(m_Drive, 3.35),
    new RotateDriveCommand(m_Drive, -90),
    new ForwardDrive(m_Drive, 1.8),
    new ShootAmpCommand(m_Intake, 750)
    );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_Drivecommand = new DefaultDriveCommand(m_Drive);
    // m_shootingcommand = new DefaultShootingCommand(m_shooter);
    
    CommandScheduler.getInstance().setDefaultCommand(m_Drive, m_Drivecommand);
    CommandScheduler.getInstance().setDefaultCommand(m_Intake, m_IntakeCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_ClimbSubsystem, m_ClimbCommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shootingcommand);

    instance = this;
  }

  public DriveSubsystem VroomVroom(){
    return m_Drive;
  }

  public Debouncer getDebouncer(){
    return limitDebouncer;
  }

  public IntakeSubsystem getIntakeSubsystem(){
    return m_Intake;
  }

  public DriveSubsystem getDriveSubsystem(){
    return m_Drive;
  }

  public ClimbSubsystem getClimbSubsystem(){
    return m_ClimbSubsystem;
  }

  public LimelightSubsystem getLimelightSubsystem(){
    return m_Limelight;
  }

  // public DriveSubsystem getLimelightDrive(){
  //   return limelightDrive;
  // }

  private void configureBindings() {
  }
  
  public void PostPosessionRoutine(){
    // CommandScheduler.getInstance().schedule(new StartingPositionsCommand(m_intake));
    CommandScheduler.getInstance().schedule(new FoldBackCommand(m_Intake));
  }

  public void AlignmentCommand(){
    CommandScheduler.getInstance().schedule(
      new LimelightAlignCommand(m_Limelight, m_Drive)
    );
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
  
  // Default Auto Command
  public Command getAutonomousCommand() {
    // switch(Robot.autoChooser.getSelected()){
      
    //   // Baseline Autonomous
    //   case(Robot.kForwardAuto):
    //     return new SequentialCommandGroup(
    //       new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(69))
    //     );
      
    //   // One Piece Amp Autonomous (DELAY)
    //   case(Robot.kPreloadAmpAuto):
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
          return new SequentialCommandGroup(
            new WaitCommand(8),
            new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(67.5)),
            new RotateDriveCommand(m_Drive, -90),
            new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(43)).withTimeout(2),
            new ShootAmpCommand(m_Intake, 750)
          );

        } else {
          
          return new SequentialCommandGroup(
            new WaitCommand(8),
            new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(67.5)),
            new RotateDriveCommand(m_Drive, 90),
            new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(43)).withTimeout(2),
            new ShootAmpCommand(m_Intake, 750)
            );
          }
        
      //   // One Piece Amp Autonomous (NO DELAY)
      //   case(Robot.kPreloadAmpAutoInstant):
      //     if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      //     return new SequentialCommandGroup(
      //       new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(69)),
      //       new RotateDriveCommand(m_Drive, -90),
      //       new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(50)),
      //       new ShootAmpCommand(m_Intake, 750)
      //     );

      //   } else {

      //     return new SequentialCommandGroup(
      //       new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(69)),
      //       new RotateDriveCommand(m_Drive, 90),
      //       new ForwardDrive(m_Drive, edu.wpi.first.math.util.Units.inchesToMeters(50)),
      //       new ShootAmpCommand(m_Intake, 750)
      //       );
      //     }
      // }

     // return null;
  }
}

