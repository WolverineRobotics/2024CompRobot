package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Input;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    public static CANSparkMax leftMotorShooter, rightMotorShooter;
    public static RelativeEncoder leftCanEncoder, rightCanEncoder;
    public static double shooter_KP = 0.05;

    public ShooterSubsystem(){
        
        leftMotorShooter = new CANSparkMax(Constants.OperatorConstants.kLeftShooterMotor1, MotorType.kBrushless);
        rightMotorShooter = new CANSparkMax(Constants.OperatorConstants.kLeftShooterMotor1, MotorType.kBrushless);

        leftMotorShooter.setIdleMode(IdleMode.kCoast);
        rightMotorShooter.setIdleMode(IdleMode.kCoast);

        leftMotorShooter.follow(rightMotorShooter);
        
        leftCanEncoder = leftMotorShooter.getEncoder();
        rightCanEncoder = rightMotorShooter.getEncoder();

        SmartDashboard.putNumber("[SHOOTER] Shooter Current", getShooterVoltage());
        SmartDashboard.putNumber("[SHOOTER] Right Shooter Velocity", leftMotorShooter.get());
        SmartDashboard.putNumber("[SHOOTER] Left Shooter Velocity", leftMotorShooter.get());
        SmartDashboard.putNumber("[SHOOTER] Shooter Position", leftCanEncoder.getPosition());

    }

    public void setShooterSpeed(){
        leftMotorShooter.set(Input.fireInTheHole());
        rightMotorShooter.set(Input.fireInTheHole());
    }

    public void setShooterVoltage(double voltage){
        leftMotorShooter.setVoltage(voltage);
        rightMotorShooter.setVoltage(voltage);
    }

    public double getShooterVoltage(){
        return leftMotorShooter.getOutputCurrent() * leftMotorShooter.getBusVoltage();
    }

    public double getShooterSpeed(){
        return leftCanEncoder.getVelocity() * Constants.shooterWheelGearRatio;
    }

    public double encoderPositions(){
        return leftCanEncoder.getPosition();
    }
    
    public double encoderRawVelocity(){
        return leftCanEncoder.getVelocity();
    }

}
