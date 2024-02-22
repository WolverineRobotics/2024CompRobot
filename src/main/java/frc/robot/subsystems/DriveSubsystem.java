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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class DriveSubsystem extends SubsystemBase {

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
  

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                _leftMaster.setVoltage(volts.in(Volts));
                _rightMaster.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            _leftMaster.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            _rightMaster.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));


  public DriveSubsystem() {

    /* Setup base drivetrain */ 
    // CANSparkMax _leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    _leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    // CANSparkMax _rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    _rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    
    // _leftFollower.follow(_leftMaster);
    // _rightFollower.follow(_rightMaster);
    
    // _leftFollower.setInverted(InvertType.FollowMaster);
    // _rightFollower.setInverted(InvertType.FollowMaster);
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

    // Unsure if integrated on victors and talons
    // leftEncoder1 = _leftMaster.getAbsoluteEncoder();
    // rightEncoder1 = _rightMaster.getAbsoluteEncoder();

    leftEncoder = _leftMaster.getEncoder();
    rightEncoder = _rightMaster.getEncoder();

    // rightEncoder.setInverted(true);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    // rightEncoder1.
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
  
  // public void setDeadband(){
  //   driveTrain.setDeadband(Constants.DEADBAND_CONST);
  // }

  // Tele-Op Driving 
  public void ArcadeDrive(){
    // driveTrain.arcadeDrive(Input.getHorizontal() * 0.3f, slew.calculate(Input.getVertical()) * 0.3f);
    driveTrain.arcadeDrive(Input.getHorizontal() * 0.4f, Input.getVertical() * 0.4f);
    

    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("Gyroscope Yaw", m_gyro.getYaw().getValueAsDouble());
  }

  // Rotate when given a speed
  public void Rotate(double speed){
    // driveTrain.arcadeDrive(speed, 0);
  }

  public Pose2d GetPose(){ return m_odometry.getPoseMeters(); }
  public Pigeon2 GetPigeon(){ return m_gyro; } 
  public double GetHeading(){ return m_pose.getRotation().getDegrees(); }

  public void AutoDrive(double speed,double rotation){
    // driveTrain.arcadeDrive(rotation, speed);
  }

  // Move straight when given a speed
  public void Straight(double speed){
    // driveTrain.arcadeDrive(speed, 0);
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

  public SysIdRoutine GetRoutine(){
    return m_sysIdRoutine;
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
