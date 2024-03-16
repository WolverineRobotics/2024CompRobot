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

public class IntakeSubsystem extends ProfiledPIDSubsystem {

    public static CANSparkMax pivotMotor, intakeMotor;
    public static RelativeEncoder pivotCanEncoder; // ,rightCanEncoder; (this variable is null)
    public static double pivot_KP = 0.05;
    private final DigitalInput intakeLimitSwitch = new DigitalInput(2);
    
    private boolean intaking;
    
    // private final DigitalInput noteLimitSwitch= new DigitalInput(Constants.kShooterLimitSwitchChannel);
    // private final DigitalInput zeroPosLimitSwitch= new DigitalInput(Constants.kShooterLimitSwitchChannel);

    public IntakeSubsystem(){

        super(
            new ProfiledPIDController(pivot_KP, 0, 0.0,
            new TrapezoidProfile.Constraints(
                OperatorConstants.kMaxIntakePivotVelocity,
                OperatorConstants.kMaxIntakePivotAcceleration)), 
            0);

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
        
        /* Checks if the subsystem has the gamepiece or not and changes robot variables accordingly */
        
        // The robot must be in full control if only the possesion switch is true 
        // if(noteLimitSwitch.get() && !Robot.has_gamepiece){
        //     Robot.has_gamepiece = true;
        //     RobotContainer.instance.AcquiredGamepiece();
        //     intaking = false;
        // }
        // else if(!noteLimitSwitch.get()){
        //     Robot.has_gamepiece = false;
        // }

        if(!Robot.has_gamepiece){ 
            if(Input.fireInTheHole() > 0){

                if(!intakeLimitSwitch.get()){ 
                    setIntakeSpeed(0);

                } else {
                     setIntakeSpeed(Input.fireInTheHole());
                }

            }
            else if(Input.AmpScore()){
                setIntakeSpeed(-1);
            }

            else{
                setIntakeSpeed(0);
            }

            pivotMotor.set(Input.Operator().getRightY() * 0.3);

            // Check if intake going into robot
            if (pivotCanEncoder.getPosition() > -3 && Input.Operator().getRightY() > 0){
                pivotMotor.set(0);
            }

            // Check if intake going into floor
            if (pivotCanEncoder.getPosition() < -39 && Input.Operator().getRightY() < 0) { //encoder value not determined
                pivotMotor.set(0);
            } 

            // if(Input.fireInTheHole() > 0 && !intaking){
            //     RobotContainer.instance.StartIntaking();
            //     intaking = true;
            // }
            
            // else if(intaking && Input.fireInTheHole() == 0){
            //     RobotContainer.instance.PostPosessionRoutine();
            //     intaking = false;
            // }
        }

        // else{
        //     if(intaking){
        //         intaking = false;
        //     }
            
        //     if(Input.driveController.getRightBumper()){
        //         setIntakeSpeed(0.95);
        //     }
        //     else{
        //         setIntakeSpeed(0);
        //     }
        // }
        SmartDashboard.putNumber("[INTAKE] Intake Current", getIntakeVoltage());
        SmartDashboard.putNumber("[INTAKE] Right Pivot Velocity", pivotMotor.get());
        SmartDashboard.putNumber("[INTAKE] Left Intake Velocity", pivotMotor.get());
        SmartDashboard.putNumber("[INTAKE] Intake Position", pivotCanEncoder.getPosition()); // Change back to pivotMotor if something is wrong
        SmartDashboard.putNumber("[INTAKE] PID Goal", getController().getGoal().position);
        SmartDashboard.putBoolean("[INTAKE] Limit Switch", intakeLimitSwitch.get());
    }
    
    
    public void setIntakeVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
        intakeMotor.setVoltage(voltage);
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
    protected void useOutput(double output, TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber("[INTAKE PID] Pivot Output", output);
        SmartDashboard.putNumber("[INTAKE PID] Pivot Setpoint", setpoint.position);

        if(!getController().atSetpoint()){ pivotMotor.set(output); }
        else{ pivotMotor.set(0); }
    };
    
    protected double getMeasurement(){
        return pivotCanEncoder.getPosition();
    };
}