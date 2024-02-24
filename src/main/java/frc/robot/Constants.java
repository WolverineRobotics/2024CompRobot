// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  
  public static final int LEFT_MOTOR_1 = 2; // Left Side (Behind the Battery (bumpers!!!!))
  public static final int LEFT_MOTOR_2 = 4;
  public static final int RIGHT_MOTOR_1 = 1; // Right Side (Behind the Battery (bumpers!!!!))
  public static final int RIGHT_MOTOR_2 = 3;

  public static final int PIGEON_ID = 3;

  /* Driving Constants */
  public static final double X_SPEEDREDUCTION = 0.6;
  public static final double Y_SPEED_REDUCTION = 0.5;
  public static final double DEADBAND_CONST = 0.05;

  /* Shooting Constants */
  public static final double shooterWheelGearRatio = 0.0;
    
  public static class OperatorConstants {

    /* Controller ports */
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    /* CAN ID constants */
    public static final int kPigeonId = 3;
    
    public static final int kLeftShooterMotor1 = 0; 
    public static final int kRightShooterMotor1 = 0; 

    public static final int kLeftMotor2 = 0; 
    public static final int kRightMotor2 = 0;

    public static final int kLeftMotorEncoder_A = 5; 
    public static final int kLeftMotorEncoder_B = 6;

    public static final int kRightMotorEncoder_A = 7; 
    public static final int kRightMotorEncoder_B = 8;

    public static final int kIntakePivotMotor = 14; 
    public static final int kRollerMotor = 10;
    
    public static final int kPivotMotor = 15;
    public static final int kIntakeMotor = 16;

    /* Trapezoidal constraints */
    public static final int kMaxElevatorVelocity = 125;
    public static final int kMaxPivotVelocity = 160;

    public static final int kMaxElevatorAcceleration = 300;
    public static final int kMaxPivotAcceleration = 300;

    public static final int kMaxDriveVelocity = 300;
    public static final int kMaxDriveAcceleration = 300;

    /* AprilTag Tracking Constraints */
    public static final double TAG_TO_ROBOT = 13;
  }

}
