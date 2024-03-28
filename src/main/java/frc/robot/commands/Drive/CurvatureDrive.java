// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CurvatureDrive extends Command {
  private final DriveSubsystem fDrive;
  
  private final PIDController rotPid = 
    new PIDController(0.03, 0, 0);

  private final ProfiledPIDController pid = 
    new ProfiledPIDController(0.4, 0.05, 0, 
    new Constraints(8, 5));

  public double initLeft;
  public double initRight;

  private boolean left, reverse;

  public double setX;
  public double setY;
  public double mainHyp;
  public double alpha, beta, addedDist, tanAlpha, travelRadius, semi_circumference, percentOfCircumference;
  public double centralDistance, leftDist, rightDist;

  public double leftWheelProportion, rightWheelProportion;

  // R stands for relative here, since these are calculated relative to the init positions
  public double RLeft;
  public double RRight;

  public double newEncoderAverage;

  public CurvatureDrive(DriveSubsystem driveSubsystem, double set_x, double set_y) {
    fDrive = driveSubsystem;

    setX = Math.abs(set_x);
    left = !(setX == set_x);
    
    setY = Math.abs(set_y); // Get setpoint
    reverse = !(setY == set_y);
    
    /* Getting robot distance to travel */
    mainHyp =  Math.abs( Math.sqrt( Math.pow(set_x, 2) + Math.pow(set_y, 2) ) ); // Get hypoteneuse (distance to setpoint)
    alpha = Math.asin(setY/mainHyp); // get alpha angle via sin opp/hyp
    tanAlpha = Math.tan(alpha); // get tan of alpha since second triangle has same theta angle as alpha
    addedDist = setY * tanAlpha; // opp of second triangle, to add to setpoint x
    travelRadius = (setX + addedDist)/2; // adds setpoint x and found opp dist to get radius of circle
    semi_circumference = Math.PI * travelRadius;

    percentOfCircumference = (90 - Math.toDegrees(alpha)) / 90;
    if(reverse){percentOfCircumference = (2 - percentOfCircumference) * -1;}

    centralDistance = semi_circumference * percentOfCircumference;
    
    /* Getting Sides distance to travel */

    double leftRad, leftCirc, leftDist;
    double rightRad, rightCirc, rightDist;
    double halfTrackWidth;
    halfTrackWidth = Constants.kDriveKinematics.trackWidthMeters / 2;
    
    
    // calculates radius' of different sides
    if(left){
      leftRad = travelRadius - halfTrackWidth;
      rightRad = travelRadius + halfTrackWidth;}
    else{
      leftRad = travelRadius + halfTrackWidth;
      rightRad = travelRadius - halfTrackWidth;}
    
    // calculates distances of different sides
    leftCirc = leftRad * Math.PI;
    leftDist = leftCirc / percentOfCircumference;
    
    rightCirc = leftRad * Math.PI;
    rightDist = rightCirc / percentOfCircumference;
    
    // ratios for wheels
    if(left){
      leftWheelProportion = leftDist / rightDist;
      rightWheelProportion = 1;}
    else{
      rightWheelProportion = rightDist / leftDist;
      leftWheelProportion = 1;}


    pid.setGoal(centralDistance);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initLeft = fDrive.getLeftEncoderPosition();
    initRight = fDrive.getRightEncoderPosition();

    pid.reset(pid.calculate(0));
    pid.setTolerance(0.5);
    // rotPid.reset();
    // rotPid.setSetpoint(fDrive.GetHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate Difference Between Current & Previous Encoder Value
    RLeft = fDrive.getLeftEncoderPosition() - initLeft;
    RRight = fDrive.getRightEncoderPosition() - initRight;

    newEncoderAverage = (RLeft+RRight)/2;
  
    double driveSpeed = pid.calculate(newEncoderAverage);
    fDrive.TankDrive(driveSpeed * leftWheelProportion, driveSpeed * rightWheelProportion);

    SmartDashboard.putNumber("SPEED OF DRIVE", driveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fDrive.AutoDrive(0, 0);
    System.out.println("AAAAAA CURVY 'Chezy' DRIVE COMMAND FINISHED EEEEEEEEEEEE");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atGoal();
    // return false;
  }
}
