// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Input;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

  public static DifferentialDrive driveTrain;
  private static DifferentialDriveKinematics m_kinematics;
  public static DifferentialDriveOdometry m_odometry;
  // public static DifferentialDrivePoseEstimator --- maybe use later
  private Pigeon2 m_gyro;
  private RelativeEncoder leftEncoder, rightEncoder;
  private Pose2d start_pose;

  private double track_width = 28;
  private double wheel_radius = 6;
  private double wheel_radius_metres = Units.inchesToMeters(wheel_radius);

  private SlewRateLimiter slew;

  public DriveSubsystem() {

    /* Setup base drivetrain */ 
    CANSparkMax _leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax _leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_2, MotorType.kBrushless);
    CANSparkMax _rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax _rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_2, MotorType.kBrushless);
    
    _leftFollower.follow(_leftMaster);
    _rightFollower.follow(_rightMaster);
    
    // _leftFollower.setInverted(InvertType.FollowMaster);
    // _rightFollower.setInverted(InvertType.FollowMaster);
    _rightMaster.setInverted(true);
    _rightFollower.setInverted(true);
    
    _leftFollower.setIdleMode(IdleMode.kBrake);
    _leftMaster.setIdleMode(IdleMode.kBrake);
    _rightFollower.setIdleMode(IdleMode.kBrake);
    _rightMaster.setIdleMode(IdleMode.kBrake);
    
    driveTrain = new DifferentialDrive(_leftMaster, _rightMaster);
    driveTrain.setSafetyEnabled(false);

    slew = new SlewRateLimiter(0.5, 0.5, 0);
    
    /* ------------------------- Setup odometry objects ------------------------- */
    /* ----------------- ENSURE EVERYTHING ODOMETRY IS IN METRES ---------------- */


    // Kinematics
    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(track_width)); /* 28 inches / 39.37 */

    // need device id
    m_gyro = new Pigeon2(0);
    m_gyro.reset();

    // Unsure if integrated on victors and talons
    leftEncoder = _rightMaster.getEncoder();
    rightEncoder = _rightMaster.getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // leftEncoder.setDistancePerPulse();
    // rightEncoder.setDistancePerPulse();

    double x = SmartDashboard.getNumber("starting_x", 0);
    double y = SmartDashboard.getNumber("starting_y", 0);

    start_pose = new Pose2d(
      new Translation2d(x, y),
      m_gyro.getRotation2d()
    );

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), start_pose);
  }
  
  // public void setDeadband(){
  //   driveTrain.setDeadband(Constants.DEADBAND_CONST);
  // }

  // Tele-Op Driving 
  public void ArcadeDrive(){
    driveTrain.arcadeDrive(Input.getHorizontal(),slew.calculate(Input.getVertical()));
    //driveTrain.arcadeDrive(Input.getHorizontal(), Input.getVertical());
  }

  // Rotate when given a speed
  public void Rotate(double speed){
    driveTrain.arcadeDrive(speed, 0);
  }

  public void AutoDrive(double speed,double rotation){
    driveTrain.arcadeDrive(rotation, speed);
  }

  // Move straight when given a speed
  public void Straight(double speed){
    driveTrain.arcadeDrive(speed, 0);
  }
  
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /* Came with the template */
    public Command exampleMethodCommand() {
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }
}
