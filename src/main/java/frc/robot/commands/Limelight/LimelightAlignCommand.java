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

    private DriveSubsystem m_Drive;
    private XboxController dController;

    private LimelightSubsystem mLimelight;

    private double m_LimelightThrottle = 0;
    private double m_LimelightTurn = 0;

    public boolean m_AprilTaginSight = false;

    public LimelightAlignCommand(LimelightSubsystem limelight){

        mLimelight = limelight;
        addRequirements(limelight);
    }

    //@Override
    /* TRACK THE TARGET */
    public void execute(){
        // trackTarget();

        // double throttle = InputSystem.DriveSpeed();
        // double turn = LimelightHelpers.getTX(""); //InputSystem.DriveRot();
        // double distance = LimelightHelpers.getTA("");
        // boolean auto = Input.alignTag();
        
        // throttle *= 0.5;
        // turn *= 0.05;

        // double limit = 0.5;

        // if(turn < -limit){turn = -limit;}
        // if(turn > limit){turn = limit;}

        // if(auto)
        // {
        //     if (m_AprilTaginSight)
        //     {
        //         if (distance >= 2){throttle = -limit;}
        //         else if(distance < 1.5){throttle = limit;}
        //         else{throttle = 0;}
        //         m_Drive.AutoDrive(-throttle, turn);
        //     } 

        //     else {m_Drive.Rotate(0);}
        //     end();
        // }

        // SmartDashboard.putNumber("[LIMELIGHT] Turn", turn);
        // SmartDashboard.putNumber("[LIMELIGHT] TX", getLimelightX());
        // SmartDashboard.putNumber("[LIMELIGHT] TY", getLimelightY());
        // SmartDashboard.putNumber("[LIMELIGHT] TA", getLimelightA());
        // SmartDashboard.putBoolean("[LIMELIGHT] TV", getLimelightV());
    }

    // /* GET LIMELIGHT VALUES */
    // private static double getLimelightX()
    // {
    //     double tx = LimelightHelpers.getTX("");
    //     return tx;
    // }

    // private static double getLimelightY()
    // {
    //     double ty = LimelightHelpers.getTY("");
    //     return ty;
    // }

    // private static double getLimelightA()
    // {
    //     double ta = LimelightHelpers.getTX("");
    //     return ta;
    // }

    // private static boolean getLimelightV()
    // {
    //     boolean tv = LimelightHelpers.getTV("");
    //     return tv;
    // }

    // /* TRACKING METHOD */
    // private void trackTarget()
    // {
    //     // Definitely needs configuration, but we ball
    //     final double DIST_BETWEEN = Constants.OperatorConstants.TAG_TO_ROBOT;
    //     final double MAX_SPEED = 0.5;
    //     final double STEER_AUTO = 0.03;
    //     final double THROTTLE_AUTO = 0.02;

    //     double tx = LimelightHelpers.getTX(""); // Horizontal Offset (-29.8 to 29.8 Degrees)
    //     double ty = LimelightHelpers.getTY(""); // Vertical Offset (-24.85 to 24.85 Degrees)
    //     double ta = LimelightHelpers.getTA(""); // Target Area (0% - 100% of the Tag)
    //     boolean tv = LimelightHelpers.getTV(""); // Valid Targeting (True/False))
        
    //     // The values in which we want the robot to turn/throttle upon detection
    //     double m_LimelightThrottle = 0;
    //     double m_LimelightTurn = 0;

    //     // When there's no AprilTag detection
    //     if (tv != true)
    //     {
    //         m_AprilTaginSight = false;
    //         m_LimelightThrottle = 0;
    //         m_LimelightTurn = 0;
    //         return;
    //     }

    //     m_AprilTaginSight = true;
        
    //     // Will continuously drive forward until the set target area has been reached
    //     double driveForward = (DIST_BETWEEN - ta) * THROTTLE_AUTO;
    //     m_LimelightThrottle = driveForward;

    //     // Will continuously rotate until it aligns
    //     double driveRotate = tx * STEER_AUTO;
    //     m_LimelightTurn = driveRotate;

    //     // Set a speed restraint while driving forward
    //     if  (m_LimelightThrottle > MAX_SPEED){m_LimelightThrottle = MAX_SPEED;} 
    // }

    public boolean isFinished(){
        // // This will try to check if the tag is within a desired error range of at least 2.5%
        // if(Math.abs(Constants.OperatorConstants.TAG_TO_ROBOT - LimelightHelpers.getTA("")) < 2.5){
        //     return true;
        // } else {
        //     return false;
        // }
        return false;
    }
    
    public void end(){
        m_LimelightThrottle = 0.0;
        m_LimelightTurn = 0.0;
    }
}
