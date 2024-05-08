package frc.robot.commands.Limelight;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;
import frc.robot.Input;
import frc.robot.AprilTagData;

public class LimelightAlignCommand extends Command{

    //private DriveSubsystem m_LimelightDrive;
    private XboxController dController;

    private LimelightSubsystem m_Limelight;
    private DriveSubsystem m_DriveSubsystem;

    private double m_LimelightThrottle;
    private double m_LimelightTurn;

    public double xSum = 0;
    public double ySum = 0;

    public boolean m_AprilTaginSight = false;
    public double[] cameraCornerCoordinates = {0, 0, 0, 0};

    public LimelightAlignCommand(LimelightSubsystem limelight, DriveSubsystem drive){

        m_Limelight = limelight;
        m_DriveSubsystem = drive;
        addRequirements(limelight, drive);
    }

    @Override
    /* TRACK THE TARGET */
    public void execute(){
        trackTarget();

        double distance = LimelightHelpers.getTA("");
        double angle = LimelightHelpers.getTX("");

        double limit = 0.35; // Turning Limit
        double throttleLimit = 0.35; // Throttle Limit 

        double limelightThrottleSetpoint = 4.3;
        double experimentalSetpoint = LimelightSubsystem.estimatedDistance(15); // The estimated distance between the tag and the robot
        double limelightThrottleError;
        double limelightAngleError;
        
        double throttle = 0.095;
        double turn = 0.05;

        if (Input.alignTag())
        {
            LimelightHelpers.setLEDMode_ForceOn("");

            if (m_AprilTaginSight)
            {
                
                /* This block applies kP control in order to try and correct the oscillations
                   and error once the robot has reached the setpoint */
                limelightThrottleError = (limelightThrottleSetpoint - distance);
                limelightAngleError = (limelightThrottleSetpoint - angle);

                if(turn < -limit){turn = -limit;}
                if(turn > limit){turn = limit;}
                if(throttle > throttleLimit){throttle = throttleLimit;}
                
                // Output = e(t) * kP
                throttle = limelightThrottleError * 0.16;

                if(limelightThrottleError < 0){
                    throttle = limelightThrottleError * 0.07;
                }

                turn = limelightAngleError * -0.017;

                // System.out.println(limelightThrottleError);
                System.out.println(limelightAngleError);
                m_DriveSubsystem.AutoDrive(throttle, turn);
            } 

        } else {
            m_DriveSubsystem.Rotate(0);
            LimelightHelpers.setLEDMode_ForceOff("");
            end();

        }

        SmartDashboard.putNumber("[LIMELIGHT] Turn", turn);
        SmartDashboard.putNumber("[LIMELIGHT] TX", getLimelightX());
        SmartDashboard.putNumber("[LIMELIGHT] TY", getLimelightY());
        SmartDashboard.putNumber("[LIMELIGHT] TA", getLimelightA());
        SmartDashboard.putBoolean("[LIMELIGHT] TV", getLimelightV());

    }

    /* TRACKING METHOD */
    private void trackTarget()
    {

        final double TAG_AREA = Constants.OperatorConstants.TAG_TO_ROBOT;
        final double MAX_SPEED = 0.1;
        final double STEER_AUTO = 0.03;
        final double THROTTLE_AUTO = 0.02;
        
        double limelight_KP = 0.035;

        double limelight_TX = LimelightHelpers.getTX(""); // Horizontal Offset (-29.8 to 29.8 Degrees)
        double limelight_TY = LimelightHelpers.getTY(""); // Vertical Offset (-24.85 to 24.85 Degrees)
        double limelight_TA = LimelightHelpers.getTA(""); // Target Area (0% - 100% of the Tag)
        boolean limelight_TV = LimelightHelpers.getTV(""); // Valid Targeting (True/False))
        
        // The values in which we want the robot to turn/throttle upon detection
        double m_LimelightThrottle = 0;
        double m_LimelightTurn = 0;

        // When there's no AprilTag detection
        if (limelight_TV != true)
        {
            m_AprilTaginSight = false;
            m_LimelightThrottle = 0;
            m_LimelightTurn = 0;
            return;
        }

        m_AprilTaginSight = true;
        
        // Will continuously drive forward until the set target area has been reached
        double driveForward = (TAG_AREA - limelight_TA) * THROTTLE_AUTO;
        m_LimelightThrottle = driveForward;

        // Will continuously rotate until it aligns
        double driveRotate = limelight_TX * STEER_AUTO;
        m_LimelightTurn = driveRotate;
    
        // Check for anything beyond a tolerance of +- 2 degrees
        if(limelight_TX > 2){
            m_LimelightTurn = (limelight_TX * limelight_KP) - STEER_AUTO;

        } else if(limelight_TX < -2){
            m_LimelightTurn = (limelight_TX * limelight_KP) + STEER_AUTO;
        }

        // Set a speed restraint while driving forward
        if (m_LimelightThrottle > MAX_SPEED){m_LimelightThrottle = MAX_SPEED;} 
    }

    /* GET LIMELIGHT VALUES */
    private static double getLimelightX()
    {
        double tx = LimelightHelpers.getTX("");
        return tx;
    }

    private static double getLimelightY()
    {
        double ty = LimelightHelpers.getTY("");
        return ty;
    }

    private static double getLimelightA()
    {
        double ta = LimelightHelpers.getTX("");
        return ta;
    }

    private static boolean getLimelightV()
    {
        boolean tv = LimelightHelpers.getTV("");
        return tv;
    }

    public boolean isFinished(){
        return true;
    }


    public double[] getCameraCenter(){
        
        // Get the x, y coordinates from the box that the limelight detects
        double[] xCoordinates = NetworkTableInstance.getDefault().getTable("").getEntry("tcornx").getDoubleArray(cameraCornerCoordinates);
        double[] yCoordinates = NetworkTableInstance.getDefault().getTable("").getEntry("tcorny").getDoubleArray(cameraCornerCoordinates);

        // For the corners of the x and y coordinates, add them together
        for(int i = 0; i < 4; ++i){
            xSum += xCoordinates[i];
            ySum += yCoordinates[i];
        }
        
        // Get the average by dividing by 4
        double[] cameraCenter = {xSum / 4, ySum / 4, 0};
        return cameraCenter;
    }
    
    public void end(){
        m_LimelightThrottle = 0.0;
        m_LimelightTurn = 0.0;
        isFinished();
    }
}