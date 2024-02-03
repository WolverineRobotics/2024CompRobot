package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.InputSystem;
import frc.robot.Constants;

public class LimelightAlignCommand extends CommandBase{

    private double target = 0;
    private DriveSubsystem m_drive;
    private XboxController dController;

    private LimelightSubsystem m_limelight;
    private double m_limelightThrottle = 0;
    private double m_limelightTurn = 0;

    public boolean m_AprilTaginSight = false;

    public LimelightAlignCommand(DriveSubsystem drive, LimelightSubsystem limelight){

        m_drive = drive;
        m_limelight = limelight;
        addRequirements(drive, limelight);
    }

    //@Override
    public void execute(){
        trackTarget();

        double throttle = InputSystem.DriveSpeed();
        double turn = LimelightHelpers.getTX(""); //InputSystem.DriveRot();
        boolean auto = InputSystem.Align();

        double distance = LimelightHelpers.getTA("");
        

        throttle *= 0.5;
        turn *= 0.05;

        double limit = 0.5;

        if(turn < -limit){
            turn = -limit;
        }
        
        if(turn > limit){
            turn = limit;
        }



        SmartDashboard.putNumber("TUUUURNNN", turn);
        SmartDashboard.putNumber("TXXX", LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("TAAA", LimelightHelpers.getTA(""));

        if(true)
        {

            if (m_AprilTaginSight){

                if(distance >= 2){
                    throttle = -limit;
                }
                else if(distance < 1.5){
                    throttle = limit;
                }
                else{
                    throttle = 0;
                }

                m_drive.AutoDrive(-throttle, turn);

            } 

            else {
                m_drive.Rotate(0); 
            }
        }

        //else {
        //    m_drive.Rotate(turn);
        //}
       
    }

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

    private void trackTarget()
    {
        
        // Definitely needs configuration, but we ball
        final double DIST_BETWEEN = Constants.OperatorConstants.TAG_TO_ROBOT;
        final double MAX_SPEED = 0.5;
        final double STEER_AUTO = 0.03;
        final double THROTTLE_AUTO = 0.02;

        double tx = LimelightHelpers.getTX(""); // Horizontal Offset (-29.8 to 29.8 Degrees)
        double ty = LimelightHelpers.getTY(""); // Vertical Offset (-24.85 to 24.85 Degrees)
        double ta = LimelightHelpers.getTA(""); // Target Area (0% - 100% of the Tag)
        boolean tv = LimelightHelpers.getTV(""); // Valid Targeting (True/False))
        
        // The values in which we want the robot to turn/throttle upon detection
        double m_limelightThrottle = 0;
        double m_limelightTurn = 0;
        
        // When there's no AprilTag detection
        if (tv != true)
        {
            m_AprilTaginSight = false;
            m_limelightThrottle = 0;
            m_limelightTurn = 0;
            return;
        }

        m_AprilTaginSight = true;
        
        // Will continuously drive forward until the set target area has been reached
        double driveForward = (DIST_BETWEEN - ta) * THROTTLE_AUTO;
        m_limelightThrottle = driveForward;

        // Will continuously rotate until it aligns
        double driveRotate = tx * STEER_AUTO;
        m_limelightTurn = driveRotate;

        // Set a speed restraint while driving forward
        if  (m_limelightThrottle > MAX_SPEED)
        {
            m_limelightThrottle = MAX_SPEED;
        } 
    }


    
    //@Override
    //protected double getMeasurement() {
    //    return pigeon.getYaw();
    //}
    
}
