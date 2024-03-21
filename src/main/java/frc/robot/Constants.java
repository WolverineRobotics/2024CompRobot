// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;

public final class Constants {
  
  public static final int LEFT_MOTOR_1 = 5; 
  public static final int LEFT_MOTOR_2 = 6;
  public static final int RIGHT_MOTOR_1 = 8; 
  public static final int RIGHT_MOTOR_2 = 7;
  
  public static final int kShooterLimitSwitchChannel = 0;
  public static final int kSPosessionLimitSwitchChannel = 1;
  public static final int kIntakePosessionLimitSwitchChannel = 1;

  public static final double kLeftDriverEncoderDistanceConversionFactor = (24/49.66) * Units.inchesToMeters(24);
  public static final double kRightDriverEncoderDistanceConversionFactor= (24/57) * Units.inchesToMeters(24);

  public static final int PIGEON_ID = 1;

  /* Driving Constants */
  public static final double X_SPEEDREDUCTION = 0.6;
  public static final double Y_SPEED_REDUCTION = 0.5;
  public static final double DEADBAND_CONST = 0.05;

  // PID & ProfiledPID Constants -- Tune Later
  public static final double kp = 0;
  public static final double ki = 0;
  public static final double kd = 0;
  
  /* Shooting Constants */
  public static final double shooterWheelGearRatio = 1.0;
  public static final double intakeWheelGearRatio = 0.0;
    
  public static class OperatorConstants {

    /* Controller ports */
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    /* CAN ID constants */
    public static final int kPigeonId = 1;
    
    public static final int kLeftShooterMotor1 = 0; 
    public static final int kRightShooterMotor1 = 0; 

    public static final int kLeftMotor2 = 0; 
    public static final int kRightMotor2 = 0;

    public static final int kIntakePivotMotor = 3; 
    public static final int kIntakeMotor = 2;

    /* Trapezoidal constraints */
    public static final int kMaxIntakePivotVelocity = 125;
    public static final int kMaxShooterPivotVelocity = 160;

    public static final int kMaxIntakePivotAcceleration = 300;
    public static final int kMaxShooterPivotAcceleration = 300;

    public static final int kMaxDriveVelocity = 300;
    public static final int kMaxDriveAcceleration = 300;

    /* AprilTag Tracking Constraints */
    public static final double TAG_TO_ROBOT = 13;
  }

  
  public static class Positional {

    // Rotate Heading Angles
    public static final int kFaceForward = 0;
    public static final int kFaceDriver = -180;
    public static final int kFaceLeft = 90;
    public static final int kFaceRight = -90;
    
    // Max and min positions for subsystems
    public static final int kShooterMaxPosition = 0;
    public static final int kIntakeMaxPosition = 215;

    public static final int kShooterMinPosition = 0;
    public static final int kIntakeMinPosition = 0;

    // For Subwoofer shot feeding and scoring 
    public static final int kShooterSubwooferShotPosition = 50;
    public static final int kIntakeSubwooferHandoffPosition = 0;
    
    // For Intaking from source(shooter) or ground (intake)
    public static final int kShooterIntakingPosition = 10;
    public static final int kIntakeIntakingPosition = -55;

    // For regular feeding to shooter 
    public static final int kShooterNonConflictPosition = 90;
    public static final int kShooterDefaultFeedPosition = 60;
    public static final int kIntakeDefaultFeedPosition = 15;
    
    // For Amp Scoring
    public static final int kShooterAmpScoringPosition = 90;
    public static final int kIntakeAmpScoringPosition = 15;

    // Limelight Mounting Position (Going by CAD here)
    public static final double limelightMountAngle = 0; // Mounted at 90 Degrees
    public static final double limelightHeight = 11.3; // From Lens --> Ground

  }

  public static class Offsets{
    public static final double kLimelightShooterOffset = 0.19;
    public static final double kSpeakerToTagHeightOffset = 0.63;
  }

}
