// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoPositionsCommands.AutoIntakeCommand;
import frc.robot.commands.AutoPositionsCommands.ShootAmpCommand;
import frc.robot.commands.Drive.ForwardDrive;
import frc.robot.commands.Drive.RotateDriveCommand;
import frc.robot.commands.Handoffs.FoldBackCommand;
import frc.robot.commands.Handoffs.FoldOutCommand;
import frc.robot.commands.Limelight.LimelightAlignCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {

  public static final String Constants = null;
  public static boolean has_gamepiece, controls_gamepiece;
  public static boolean isFoldedBack, pivotIsMoving;
  public static int foldedTimer = 0;
  
  public static final String kForwardAuto = "Default Auto";
  public static final String kPreloadAmpAuto = "Preload Amp Auto (Delayed)";
  public static final String kPreloadAmpAutoInstant = "Preload Amp Auto (No Delay)";
  public static String m_AutoChosen;
  public static SendableChooser<String> autoChooser = new SendableChooser<>();

public static Object subsystems;

private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();

    // autoChooser.setDefaultOption("Default Auto", kForwardAuto);
    // autoChooser.addOption("Amp Auto (Delay)", kPreloadAmpAuto);
    // autoChooser.addOption("Amp Auto (No Delay)", kPreloadAmpAutoInstant);
    // SmartDashboard.putData("Auto Choices", autoChooser);
    
    // Set to false if there is not in fact a gamepiece on starting config.
    has_gamepiece = false;
    // Properly carrying the gamepiece
    controls_gamepiece = false;
    isFoldedBack = true;
    pivotIsMoving = false;

    SmartDashboard.putBoolean("[INTAKE] IS FOLDED BACK", isFoldedBack);
  }

  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();



    // if(foldedTimer > 0){
    //   foldedTimer -= 20;
    //   isFoldedBack = false;
    // }
    // else if(!isFoldedBack && !pivotIsMoving){
    //   // isFoldedBack = true;
    //   m_robotContainer.getIntakeSubsystem().ResetPivotEncoder();
    // }

    if(Input.opController.getAButtonPressed()) {m_robotContainer.getIntakeSubsystem().ResetPivotEncoder();}
  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LimelightHelpers.setLEDMode_ForceOff("");
    
  }
  
  @Override
  public void disabledPeriodic() {}
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.VroomVroom().ResetGyro();

    //CommandScheduler.getInstance().schedule(new ForwardDrive(m_robotContainer.VroomVroom()));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // m_AutoChosen = autoChooser.getSelected();
    // System.out.println("Auto Selected: " + m_AutoChosen);
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    } 
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    LimelightHelpers.setLEDMode_ForceOff("");
    m_robotContainer.VroomVroom().GetPigeon().reset();
    
  }
  
  public static void ResetFoldedBackTimer(){
    foldedTimer = 1000;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(Input.opController.getRightBumperPressed()){
      new FoldOutCommand(m_robotContainer.getIntakeSubsystem()).
      andThen(new AutoIntakeCommand(m_robotContainer.getIntakeSubsystem())).schedule();
    }

    if(Input.opController.getRightBumperReleased()){
      new FoldBackCommand(m_robotContainer.getIntakeSubsystem()).schedule();
    }

    if(Input.driveController.getAButtonPressed()){
      m_robotContainer.VroomVroom().GetPigeon().setYaw(0);
    }
    
    if(Input.driveController.getXButton()){
      new LimelightAlignCommand(m_robotContainer.getLimelightSubsystem(), m_robotContainer.getDriveSubsystem()).schedule();

    }
    /*if(Input.alignTag()){
      new LimelightAlignCommand(m_robotContainer.getLimelightSubsystem(), m_robotContainer.getLimelightDrive()).schedule();
    }*/
  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public RobotContainer GetContainer() {return m_robotContainer;}
}
