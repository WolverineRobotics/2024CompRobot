package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends ProfiledPIDSubsystem {

    // Default table key is "limelight"
    //NetworkTable m_Table = NetworkTableInstance.getDefault().getTable("limelight");
    
    private double tx, ty, ta;

    private final static double Kp = 0.03;
    private final static double Ki = 0;
    private final static double Kd = 0;
    
    private double pid_output = 0;
    private double pipeline_latency = 0;
    private double capture_latency = 0;

    private double tag_id = 0;
    
    public LimelightSubsystem(){
        super(
           new ProfiledPIDController(Kp, Ki, Kd,
           new TrapezoidProfile.Constraints(
               OperatorConstants.kMaxDriveVelocity,
               OperatorConstants.kMaxDriveAcceleration
               )
            )
        );


        // Smart dashboard assignments
        SmartDashboard.putNumber("LL_tx", tx);
        SmartDashboard.putNumber("LL_ty", ty);
        SmartDashboard.putNumber("LL_ta", ta);
        
        SmartDashboard.putNumber("LL_capture_ping", capture_latency);
        SmartDashboard.putNumber("LL_pipeline_ping", pipeline_latency);
        SmartDashboard.putNumber("LL_AprilTag_id", tag_id);
        
        
        setDefaultCommand(null);
    }
    
    @Override
    public void periodic(){
        
        // Default table key is "limelight"

        // Read limelight values"
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        
        // Latencies
        capture_latency = LimelightHelpers.getLatency_Capture("");
        pipeline_latency = LimelightHelpers.getLatency_Pipeline("");
        
        // detected April tag value
        tag_id = LimelightHelpers.getFiducialID("");

        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices
        // Explains that we need a static IP configuration before connecting to the field
    }

    // Reads the auto alignment PID to the pid_output variable
    @Override
    protected void useOutput(double output, edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {
        pid_output = output;
    }

    public double ReadRotationalOutput(){
        return pid_output;
    }
    
    @Override
    protected double getMeasurement() {
        return tx;
    }

}
