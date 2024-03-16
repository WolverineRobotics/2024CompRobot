package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends ProfiledPIDSubsystem {

    // Default table key is "limelight"
    //NetworkTable m_Table = NetworkTableInstance.getDefault().getTable("limelight");
    
    private double tx, ty, ta;

    private final static double Kp = 0.03;
    private final static double Ki = 0;
    private final static double Kd = 0;
    
    private double pidOutput = 0;
    private double pipelineLatency = 0;
    private double captureLatency = 0;

    private double tagId = 0;
    
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
        
        SmartDashboard.putNumber("LL_capture_ping", captureLatency);
        SmartDashboard.putNumber("LL_pipeline_ping", pipelineLatency);
        SmartDashboard.putNumber("LL_AprilTag_id", tagId);
        
        
        setDefaultCommand(null);
    }

    /* DISTANCE CALCULATION 
       distance = (height of target - height of camera) / tan(mounting angle + angle to target)

       What I'm thinking right now is to somehow pass target height values into this method and calculate an estimated distance...
       TODO: Get Measurements
    */

    private double estimatedDistance(double goalHeightMeters){
        double tagAngle = LimelightHelpers.getTY("");

        double limelightOffsetAngle = Constants.Positional.limelightMountAngle; // Angular offset vertically
        double limelightHeightMeters = Constants.Positional.limelightHeight; // Height from ground -> lens
        double goalAngle = Math.toRadians(limelightOffsetAngle + tagAngle);

        return (goalHeightMeters - limelightHeightMeters) / (Math.tan(goalAngle));
    }
    
    @Override
    public void periodic(){
        
        // Default table key is "limelight"

        // Read limelight values"
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");

        //LimelightHelpers.getBotPose2d("");
        
        // Latencies
        captureLatency = LimelightHelpers.getLatency_Capture("");
        pipelineLatency = LimelightHelpers.getLatency_Pipeline("");


        //LimelightHelpers.getBotPose2d("");
        
        // detected April tag value
        tagId = LimelightHelpers.getFiducialID("");

        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices
        // Explains that we need a static IP configuration before connecting to the field
    }

    public Pose2d TryPose(){
        Pose2d estimatedPose;
        if(LimelightHelpers.getTV("")){
            estimatedPose = LimelightHelpers.getBotPose2d("");
        }

        return null;
    }

    // Reads the auto alignment PID to the pid_output variable
    @Override
    protected void useOutput(double output, edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {
        pidOutput = output;
    }

    public double ReadRotationalOutput(){
        return pidOutput;
    }
    
    @Override
    protected double getMeasurement() {
        return tx;
    }

}
