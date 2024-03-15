// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
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

public class DriveSubsystem extends ProfiledPIDSubsystem {

  public static DifferentialDrive driveTrain;
  private static DifferentialDriveKinematics m_kinematics;
  public static DifferentialDriveOdometry m_odometry;
  // public static DifferentialDrivePoseEstimator --- maybe use later
  private Pigeon2 m_gyro;
  private RelativeEncoder leftEncoder, rightEncoder;
  //private SparkAbsoluteEncoder leftEncoder1, rightEncoder1;
  //private final Encoder _l = 
  private Pose2d start_pose, m_pose;

  private CANSparkMax _leftMaster;//= new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax _rightMaster;// = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);

  private double track_width = 25;
  private double wheel_radius = 3;
  private double wheel_radius_metres = Units.inchesToMeters(wheel_radius);

  private final PIDController left_pid = new PIDController(0.1, 0.03, 0.05);
  private final PIDController right_pid = new PIDController(0.1, 0.03, 0.05);
  private final RamseteController m_RamseteController = new RamseteController();

  private SlewRateLimiter slew;


  public DriveSubsystem() {
    // Rotate In Place Gains & Goals - Separate PID Controller For DriveForward
    super(
      new ProfiledPIDController(Constants.kp, Constants.ki, Constants.kd, 
      new TrapezoidProfile.Constraints(OperatorConstants.kMaxDriveVelocity, OperatorConstants.kMaxDriveAcceleration)), 0
    );
 
    // Set Tolerance For Rotating 
    getController().setTolerance(15);

    /* Setup base drivetrain */ 
    // CANSparkMax _leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    _leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    // CANSparkMax _rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    _rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    
    // _leftFollower.follow(_leftMaster);
    // _rightFollower.follow(_rightMaster);

    
    _rightMaster.setInverted(true);
    _leftMaster.setInverted(true);
    
    _leftMaster.setIdleMode(IdleMode.kBrake);
    _rightMaster.setIdleMode(IdleMode.kBrake);
    
    // _leftFollower.setIdleMode(IdleMode.kBrake);
    // _rightFollower.setIdleMode(IdleMode.kBrake);
    
    driveTrain = new DifferentialDrive(_leftMaster, _rightMaster);
    driveTrain.setSafetyEnabled(false);

    // slew = new SlewRateLimiter(0.5, 0.5, 0);
    
    /* ------------------------- Setup odometry objects ------------------------- */
/* ----------------- ENSURE EVERYTHING ODOMETRY IS IN METRES ---------------- */
    
    
    // Kinematics
    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(track_width));
    
    // need device id
    m_gyro = new Pigeon2(Constants.PIGEON_ID);
    m_gyro.reset();
    
    leftEncoder = _leftMaster.getEncoder();
    rightEncoder = _rightMaster.getEncoder();
    
    // rightEncoder.setInverted(true);
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    float left_counts_per_rev = leftEncoder.getCountsPerRevolution();
    float right_counts_per_rev = rightEncoder.getCountsPerRevolution();
    leftEncoder.setPositionConversionFactor(6);
    rightEncoder.setPositionConversionFactor(6);
    
    double x = 0;
    double y = 0;
    // double x = SmartDashboard.getNumber("starting_x", 0);
    // double y = SmartDashboard.getNumber("starting_y", 0);
    
    start_pose = new Pose2d(
      new Translation2d(x, y),
      m_gyro.getRotation2d()
    );
      
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), start_pose);
    }

    // Profiled PID Commands
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
      SmartDashboard.putNumber("[DRIVE] Output", output);
      SmartDashboard.putNumber("[DRIVE] Setpoint", setpoint.position);
      if(!getController().atGoal()) {
        AutoDrive(0, output);
      }
    }

    // Get Pigeon Heading
    protected double getMeasurement() {
      SmartDashboard.putNumber("[DRIVE] Pigeon Heading", GetHeading());
      return GetHeading();
    }
    
    // public void setDeadband(){
  //   driveTrain.setDeadband(Constants.DEADBAND_CONST);
  // }

  // Tele-Op Driving 
  public void ArcadeDrive(){
    // driveTrain.arcadeDrive(Input.getHorizontal() * 0.3f, slew.calculate(Input.getVertical()) * 0.3f);
    driveTrain.arcadeDrive(Input.getHorizontal() * 0.4f, Input.getVertical() * 0.4f);
    
    // SmartDashboard.putNumber("[DRIVE] Left Encoder", leftEncoder.getPosition());
    // SmartDashboard.putNumber("[DRIVE] Right Encoder", rightEncoder.getPosition());
    // SmartDashboard.putNumber("[DRIVE] Gyroscope Yaw", m_gyro.getYaw().getValueAsDouble());
  }

  public Pose2d GetPose(){ return m_odometry.getPoseMeters(); }
  public Pigeon2 GetPigeon(){ return m_gyro; } 
  public double GetHeading(){ return m_pose.getRotation().getDegrees();}

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
      leftEncoder.getVelocity());
    }
    
    public void SetDriveVoltages(double l_volts, double r_volts){
      _rightMaster.setVoltage(r_volts);
      _leftMaster.setVoltage(l_volts);
      driveTrain.feed();
    }
    
    public void ResetGyro(){
      m_gyro.reset();
    }
    
    public double GetTurnRate(){
      return m_gyro.getRate();
    }
    
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      m_pose = m_odometry.update(m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    
    /* Came with the template */
    public Command exampleMethodCommand() {
      return runOnce(
        () -> {
          /* one-time action goes here */
        });
      }
}
