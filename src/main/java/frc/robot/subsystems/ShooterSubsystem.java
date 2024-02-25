package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Input;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends ProfiledPIDSubsystem {

    public static CANSparkMax rightPivotMotor, leftShooterMotor, posessionMotor;
    public static RelativeEncoder pivotCanEncoder, rightCanEncoder;
    public static double shooter_KP = 0.05;

    public final DigitalInput shooterLimitSwitch = new DigitalInput(Constants.kShooterLimitSwitchChannel);
    public final DigitalInput posessionLimitSwitch = new DigitalInput(Constants.kSPosessionLimitSwitchChannel);

    public ShooterSubsystem(){

        super(
            new ProfiledPIDController(shooter_KP, 0, 0.0,
            new TrapezoidProfile.Constraints(
                OperatorConstants.kMaxShooterPivotVelocity,
                OperatorConstants.kMaxShooterPivotAcceleration)), 
            0);

        // Initializing motors
        leftShooterMotor = new CANSparkMax(Constants.OperatorConstants.kLeftShooterMotor1, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(Constants.OperatorConstants.kRightShooterMotor1, MotorType.kBrushless);
        posessionMotor = new CANSparkMax(Constants.OperatorConstants.kRightShooterMotor1, MotorType.kBrushless);
        
        // Initializing Encoders and PID starting config
        
        //pivotCanEncoder.setInverted(true);
        pivotCanEncoder = rightPivotMotor.getEncoder();
        pivotCanEncoder.setPosition(0);
        setGoal(-10);
        getController().setTolerance(2);
        
        
        // Initializing Idle Modes for Motors
        rightPivotMotor.setIdleMode(IdleMode.kBrake);
        leftShooterMotor.setIdleMode(IdleMode.kCoast);
        posessionMotor.setIdleMode(IdleMode.kBrake);
        
        
        // SmartDashboard logging
        SmartDashboard.putNumber("[SHOOTER] Shooter Current", getShooterVoltage());
        SmartDashboard.putNumber("[SHOOTER] Right Pivot Velocity", rightPivotMotor.get());
        SmartDashboard.putNumber("[SHOOTER] Left Shooter Velocity", rightPivotMotor.get());
        SmartDashboard.putNumber("[SHOOTER] Shooter Position", rightCanEncoder.getPosition());

    }


    @Override
    public void periodic(){

        /* Checks if the subsystem has the gamepiece or not and changes robot variables accordingly */

        // The robot must be in full control if only the possesion switch is true 
        if(posessionLimitSwitch.get() && !shooterLimitSwitch.get()){
            Robot.controls_gamepiece = true; }

        // The robot must have the gamepiece if either switches are true 
        if(posessionLimitSwitch.get() || shooterLimitSwitch.get() && !Robot.has_gamepiece){
            Robot.has_gamepiece = true; }
            
        // The robot must have lost or shot the gamepiece if both switches are false and control is still marked as true 
        if(!posessionLimitSwitch.get() && !shooterLimitSwitch.get() && Robot.controls_gamepiece){ 
            Robot.controls_gamepiece = false;
            Robot.has_gamepiece = false;

            RobotContainer.instance.PostPosessionRoutine();
        }

        if(Robot.controls_gamepiece){ setShooterSpeed(Input.fireInTheHole()); }
        else if (!Robot.has_gamepiece) { setShooterSpeed(Input.fireInTheHole() * -0.2); }
    }


    public void setShooterSpeed(double speed){ leftShooterMotor.set(speed); }

    public void setPosessionSpeed(double speed){ posessionMotor.set(speed); }

    public void setShooterVoltage(double voltage){
        rightPivotMotor.setVoltage(voltage);
        leftShooterMotor.setVoltage(voltage);
    }

    public double getShooterVoltage(){
        return rightPivotMotor.getOutputCurrent() * rightPivotMotor.getBusVoltage();
    }

    public double getShooterSpeed(){
        return pivotCanEncoder.getVelocity() * Constants.shooterWheelGearRatio;
    }

    public double encoderPositions(){
        return pivotCanEncoder.getPosition();
    }
    
    public double encoderRawVelocity(){
        return pivotCanEncoder.getVelocity();
    }

    // Profiled PID derived methods
    protected void useOutput(double output, TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber("pivot_output", output);
        SmartDashboard.putNumber("pivot setpoint", setpoint.position);

        if(!getController().atSetpoint()){ rightPivotMotor.set(output); }
        else{ rightPivotMotor.set(0); }
    };
    
    protected double getMeasurement(){
        return pivotCanEncoder.getPosition();
    };
}
