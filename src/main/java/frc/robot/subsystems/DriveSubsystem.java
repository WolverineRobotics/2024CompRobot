// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Input;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

  public static DifferentialDrive driveTrain;
  private static DifferentialDriveKinematics mKinematics;
  public static DifferentialDriveOdometry mOdometry;
  // public static DifferentialDrivePoseEstimator --- maybe use later
  private Pigeon2 mGyro;
  private RelativeEncoder leftEncoder, rightEncoder;
  //private SparkAbsoluteEncoder leftEncoder1, rightEncoder1;
  //private final Encoder _l = 
  private Pose2d startPose, mPose;

  private CANSparkMax leftMaster;//= new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax rightMaster;// = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);

  private double trackWidth = 25;
  private double wheelRadius = 3;
  private double wheelRadiusMeters = Units.inchesToMeters(wheelRadius);
  private double encoderPositionAverage;

  private final PIDController left_pid = new PIDController(0.1, 0.03, 0.05);
  private final PIDController right_pid = new PIDController(0.1, 0.03, 0.05);
  private final RamseteController m_RamseteController = new RamseteController();

  private SlewRateLimiter slew;


  public DriveSubsystem() {

    /* Setup base drivetrain */ 
    leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_2, MotorType.kBrushless);

    rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_2, MotorType.kBrushless);
    
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    
    // leftFollower.setInverted(InvertType.FollowMaster);
    // rightFollower.setInverted(InvertType.FollowMaster);

    rightMaster.setInverted(true);
    leftMaster.setInverted(true);
    
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    
    driveTrain = new DifferentialDrive(leftMaster, rightMaster);
    driveTrain.setSafetyEnabled(false);

    // slew = new SlewRateLimiter(0.5, 0.5, 0);
    
    /* ------------------------- Setup odometry objects ------------------------- */
/* ----------------- ENSURE EVERYTHING ODOMETRY IS IN METRES ---------------- */
    
    
    // Kinematics
    mKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
    
    // need device id
    mGyro = new Pigeon2(Constants.PIGEON_ID);
    mGyro.reset();
    
    // Unsure if integrated on victors and talons
    // leftEncoder1 = _leftMaster.getAbsoluteEncoder();
    // rightEncoder1 = _rightMaster.getAbsoluteEncoder();
    
    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
    
    // rightEncoder.setInverted(true);
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    float left_counts_per_rev = leftEncoder.getCountsPerRevolution();
    float right_counts_per_rev = rightEncoder.getCountsPerRevolution();
    
    leftEncoder.setPositionConversionFactor(Constants.kDriverEncoderDistanceConversionFactor);
    rightEncoder.setPositionConversionFactor(Constants.kDriverEncoderDistanceConversionFactor);
    
    double x = 0;
    double y = 0;
    // double x = SmartDashboard.getNumber("starting_x", 0);
    // double y = SmartDashboard.getNumber("starting_y", 0);
    
    startPose = new Pose2d(
      new Translation2d(x, y),
      mGyro.getRotation2d()
    );
      
    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), startPose);
        }
    
    // public void setDeadband(){
  //   driveTrain.setDeadband(Constants.DEADBAND_CONST);
  // }

  // Tele-Op Driving 
  public void ArcadeDrive(){
    // driveTrain.arcadeDrive(Input.getHorizontal() * 0.3f, slew.calculate(Input.getVertical()) * 0.3f);
    driveTrain.arcadeDrive(Input.getHorizontal() * 0.f, Input.getVertical());
    
    // SmartDashboard.putNumber("[DRIVE] Left Encoder", leftEncoder.getPosition());
    // SmartDashboard.putNumber("[DRIVE] Right Encoder", rightEncoder.getPosition());
    // SmartDashboard.putNumber("[DRIVE] Gyroscope Yaw", m_gyro.getYaw().getValueAsDouble());
  }

  public Pose2d GetPose(){ return mOdometry.getPoseMeters(); }
  public Pigeon2 GetPigeon(){ return mGyro; } 
  public double GetHeading(){ return mPose.getRotation().getDegrees();}

  // if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
  //   new DifferentialDrivePoseEstimator(
  //     m_kinematics, 
  //     m_gyro.getRotation2d(),
  //     leftEncoder.getDistance(), 
  //     rightEncoder.getDistance() 
  //     m_pose);
  // }

  public void AutoDrive(double speed,double rotation){
      // driveTrain.arcadeDrive(rotation, speed);
    }

  public void Rotate(double rotation){
    // driveTrain.arcadeDrive(rotation, 0);
  }
  
  public DifferentialDriveWheelSpeeds GetWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      rightEncoder.getVelocity(),
      leftEncoder.getVelocity()
      );
  }
  
  public void SetDriveVoltages(double l_volts, double r_volts){
    rightMaster.setVoltage(r_volts);
    leftMaster.setVoltage(l_volts);
    driveTrain.feed();
  }
  
  public void ResetGyro(){
    mGyro.reset();
  }
  
  public double GetTurnRate(){
    return mGyro.getRate();
  }
  
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      mPose = mOdometry.update(mGyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    
    /* Came with the template */
    public Command exampleMethodCommand() {
      return runOnce(
        () -> {
          /* one-time action goes here */
        });
      }
}
