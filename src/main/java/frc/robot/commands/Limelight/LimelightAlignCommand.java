package frc.robot.commands.Limelight;

import edu.wpi.first.apriltag.AprilTag;
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

    private DriveSubsystem m_LimelightDrive;
    private XboxController dController;

    private LimelightSubsystem m_Limelight;
    private DriveSubsystem m_DriveSubsystem;

    private double m_LimelightThrottle;
    private double m_LimelightTurn;

    public boolean m_AprilTaginSight = false;

    public LimelightAlignCommand(LimelightSubsystem limelight, DriveSubsystem drive){

        m_Limelight = limelight;
        m_LimelightDrive = drive;
        addRequirements(limelight, drive);
    }

    @Override
    /* TRACK THE TARGET */
    public void execute(){
        trackTarget();

        double throttle = Input.getVertical();
        double turn = Input.getHorizontal();

        double distance = LimelightHelpers.getTA("");
        double limit = 0.5;
        
        throttle *= 0.5;
        turn *= 0.05;

        if(turn < -limit){turn = -limit;}
        if(turn > limit){turn = limit;}

        if(Input.alignTag())
        {
            LimelightHelpers.setLEDMode_ForceOn("");

            if (m_AprilTaginSight)
            {
                if (distance >= 2){throttle = -limit;}
                else if(distance < 1.5){throttle = limit;}
                else{throttle = 0;}
                m_LimelightDrive.AutoDrive(-throttle, turn);
            } 

            else {m_LimelightDrive.Rotate(0);}
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
        // Definitely needs configuration, but we ball
        final double DIST_BETWEEN = Constants.OperatorConstants.TAG_TO_ROBOT;
        final double MAX_SPEED = 0.5;
        final double STEER_AUTO = 0.03;
        final double THROTTLE_AUTO = 0.02;
        
        double limelight_KP = 0.05;

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
        double driveForward = (DIST_BETWEEN - limelight_TA) * THROTTLE_AUTO;
        m_LimelightThrottle = driveForward;

        // Will continuously rotate until it aligns
        double driveRotate = limelight_TX * STEER_AUTO;
        m_LimelightTurn = driveRotate;
    
        // Check for anything beyond a tolerance of +- 1.5 degrees
        if(limelight_TX > 1.5){
            m_LimelightTurn = (limelight_TX * limelight_KP) - STEER_AUTO;

        } else if(limelight_TX < -1.5){
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
        // This will try to check if the tag is within a desired error range of at least 2.5%
        if(Math.abs(Constants.OperatorConstants.TAG_TO_ROBOT - LimelightHelpers.getTA("")) < 2.5){
            return true;
        } else {
            return false;
        }
    }
    
    public void end(){
        m_LimelightThrottle = 0.0;
        m_LimelightTurn = 0.0;
    }
}