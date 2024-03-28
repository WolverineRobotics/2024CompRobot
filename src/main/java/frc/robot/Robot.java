// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoPositionsCommands.AutoIntakeCommand;
import frc.robot.commands.AutoPositionsCommands.FoldBackCommand;
import frc.robot.commands.AutoPositionsCommands.FoldOutCommand;
import frc.robot.commands.Drive.ForwardDrive;

public class Robot extends TimedRobot {
  public static final String Constants = null;
  public static boolean has_gamepiece, controls_gamepiece;
  public static boolean isFoldedBack, pivotIsMoving;
  public static int foldedTimer = 0;

public static Object subsystems;

private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    
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
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    }

    
    
    // LimelightHelpers.setLEDMode_ForceOn("");
    
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
