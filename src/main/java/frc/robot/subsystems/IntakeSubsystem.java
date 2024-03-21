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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends PIDSubsystem {

    public final CANSparkMax pivotMotor, intakeMotor;
    public final RelativeEncoder pivotCanEncoder; // ,rightCanEncoder; (this variable is null)
    public static double pivot_KP = 0.1;
    public static double pivot_KI = 0;
    public static double pivot_KD = 0;
    public final DigitalInput intakeLimitSwitch = new DigitalInput(2);
    

    private boolean intaking;
    
    // private final DigitalInput noteLimitSwitch= new DigitalInput(Constants.kShooterLimitSwitchChannel);
    // private final DigitalInput zeroPosLimitSwitch= new DigitalInput(Constants.kShooterLimitSwitchChannel);

    public IntakeSubsystem(){

        super(new PIDController(pivot_KP, pivot_KI, pivot_KD));

        // Initializing motors
        intakeMotor = new CANSparkMax(Constants.OperatorConstants.kIntakeMotor, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.OperatorConstants.kIntakePivotMotor, MotorType.kBrushless);
        
        // Initializing Encoders and PID starting config
        
        //pivotCanEncoder.setInverted(true);

        pivotCanEncoder = pivotMotor.getEncoder();
        pivotCanEncoder.setPosition(0);
        pivotCanEncoder.setPositionConversionFactor(1);
        pivotCanEncoder.setVelocityConversionFactor(1);

        // setGoal(10);
        getController().setTolerance(2);
        
        // Initializing Idle Modes for Motors
        pivotMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        
        intaking = false;
    }
    
    
    @Override
    public void periodic(){
        // super.periodic();
        
        SmartDashboard.putNumber("[INTAKE] Intake Current", getIntakeVoltage());
        SmartDashboard.putNumber("[INTAKE] Right Pivot Velocity", pivotMotor.get());
        SmartDashboard.putNumber("[INTAKE] Left Intake Velocity", pivotMotor.get());
        SmartDashboard.putNumber("[INTAKE] Intake Position", pivotCanEncoder.getPosition()); // Change back to pivotMotor if something is wrong
        SmartDashboard.putNumber("[INTAKE] PID Goal", getController().getSetpoint());
        SmartDashboard.putBoolean("[INTAKE] Limit Switch", intakeLimitSwitch.get());
        
        // useOutput(0, getController().getSetpoint());
        
    }
    
    
    public void setIntakeVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
        intakeMotor.setVoltage(voltage);
    }
    
    public boolean atSetpoint(){
        return getController().atSetpoint();
    }
    
    public void setIntakeSpeed(double speed){ intakeMotor.set(speed); }
    
    /* I'm not sure why we have methods for brake and coasting?? */
    // public void Brake(){ intakeMotor.setIdleMode(IdleMode.kBrake); }
    // public void Coast(){ intakeMotor.setIdleMode(IdleMode.kCoast); }

    public double getIntakeVoltage(){ return pivotMotor.getOutputCurrent() * pivotMotor.getBusVoltage();}

    public double getIntakeSpeed(){ return pivotCanEncoder.getVelocity() * Constants.shooterWheelGearRatio; }

    public double getEncoderPosition(){ return pivotCanEncoder.getPosition(); }

    public double getEncoderRawVelocity(){ return pivotCanEncoder.getVelocity(); }



    // Profiled PID derived methods
    @Override
    protected void useOutput(double output, double setpoint){
        SmartDashboard.putNumber("[INTAKE PID] Pivot Output", output);
        SmartDashboard.putNumber("[INTAKE PID] Pivot Setpoint", setpoint);

       pivotMotor.set(output + m_controller.calculate(setpoint));
    };
    
    protected double getMeasurement(){
        return pivotCanEncoder.getPosition();
    };

    public void setIntakeSetpoint(double setPoint){
        getController().setSetpoint(setPoint);
        enable();
    }

    
}