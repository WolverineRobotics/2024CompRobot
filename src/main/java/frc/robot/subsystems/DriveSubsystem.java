// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Input;

public class DriveSubsystem extends ProfiledPIDSubsystem {

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

  // private final PIDController left_pid = new PIDController(0.1, 0.03, 0.05);
  // private final PIDController right_pid = new PIDController(0.1, 0.03, 0.05);
  // private final RamseteController m_RamseteController = new RamseteController();

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
    leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_2, MotorType.kBrushless);

    rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_2, MotorType.kBrushless);
    
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    
    rightMaster.setInverted(true);
    leftMaster.setInverted(false);
    
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    
    driveTrain = new DifferentialDrive(leftMaster, rightMaster);
    driveTrain.setDeadband(0.1);
    driveTrain.setSafetyEnabled(false);
    // slew = new SlewRateLimiter(0.5, 0.5, 0);
    
    /* ------------------------- Setup odometry objects ------------------------- */
    /* ----------------- ENSURE EVERYTHING ODOMETRY IS IN METRES ---------------- */
    
    // Kinematics
    mKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
    
    // need device id
    mGyro = new Pigeon2(Constants.PIGEON_ID);
    mGyro.reset();
    
    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    // rightEncoder.setInverted(true);
    
    leftEncoder.setPositionConversionFactor((0.127) * Units.inchesToMeters(24));
    rightEncoder.setPositionConversionFactor((0.127) * Units.inchesToMeters(24));
    
    double x = 0;
    double y = 0;
    // double x = SmartDashboard.getNumber("starting_x", 0);
    // double y = SmartDashboard.getNumber("starting_y", 0);

    SmartDashboard.putNumber("left_RATE", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right_RATE", rightEncoder.getVelocity());
    
    startPose = new Pose2d(
      new Translation2d(x, y),
      mGyro.getRotation2d()
    );
     
    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition(), startPose);
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

  // Tele-Op Driving 
  public void ArcadeDrive(){
    driveTrain.arcadeDrive(Input.getVertical(), Input.getHorizontal() * 0.8f);
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
      driveTrain.arcadeDrive(rotation, speed);
    }

  public void Rotate(double rotation){
    driveTrain.arcadeDrive(rotation, 0);
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
    mPose = mOdometry.update(mGyro.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition());
    
    SmartDashboard.putNumber("[DRIVE] Left Distance ", leftEncoder.getPosition());
    SmartDashboard.putNumber("[DRIVE] Right Distance ", -rightEncoder.getPosition());

    // SmartDashboard.putNumber("[DRIVE] Left Ticks ", leftEncoder.());
    // SmartDashboard.putNumber("[DRIVE] Right Ticks ", -rightEncoder.());

    SmartDashboard.putNumber("[DRIVE] Left Counts per Revolution", leftEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("[DRIVE] Right Counts per Revolution", rightEncoder.getCountsPerRevolution());
    
    SmartDashboard.putNumber("[ODOMETRY] x", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("[ODOMETRY] y", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("[ODOMETRY] rotation", mGyro.getAngle());

    if(Input.driveController.getXButton()){
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }

    // if(Input.driveController.getYButtonPressed()){
    //   sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
    // }

    // if(Input.driveController.getYButtonReleased()){
    //   getCurrentCommand().cancel();
    // }
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.quasistatic(direction);
  // }

  // /**
  //  * Returns a command that will execute a dynamic test in the given direction.
  //  *
  //  * @param direction The direction (forward or reverse) to run the test in
  //  */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.dynamic(direction);
  // }

  /* Came with the template */
  public Command exampleMethodCommand() {
    return runOnce(
      () -> {
        /* one-time action goes here */
      });
    }
}
