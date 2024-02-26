package frc.robot.commands.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.InputSystem;
import frc.robot.Constants;
import frc.robot.Input;

public class ShooterLimelightCommand extends Command{

    private double target = 0;
    private ShooterSubsystem m_shooter;
    private XboxController dController;

    private LimelightSubsystem m_limelight;
    
    private Double[] target_pos;
    private Double shooter_dist_to_target, shooter_angle, pid_goal;
    private Pose3d target_pose;

    public boolean m_AprilTaginSight = false, at_goal;

    public ShooterLimelightCommand(ShooterSubsystem shooter, LimelightSubsystem limelight){

        m_shooter = shooter;
        m_limelight = limelight;
        addRequirements(shooter, limelight);
    }

    /* TRACK THE TARGET */
    @Override
    public void execute(){
        if(LimelightHelpers.getTV("")){
            if(LimelightHelpers.getFiducialID("") == target){
                // If limelight sees shooter target
                
                // Setting indication and rumble
                if(m_shooter.getController().atGoal() && !at_goal){
                    Input.Operator().setRumble(RumbleType.kBothRumble, 0.3);
                }
                else{
                    at_goal = false;
                    Input.Operator().setRumble(RumbleType.kBothRumble, 0);
                }
                
                // Distances are in metres
                // Getting initial limelights target pose
                target_pose = LimelightHelpers.getTargetPose3d_CameraSpace("");
                target_pos = new Double[]{
                    target_pose.getX(),
                    target_pose.getY(),
                    target_pose.getZ(),
                };

                // applying offsets for accurate target pose
                target_pos[2] -= Constants.Offsets.kLimelightShooterOffset;
                target_pos[2] += Constants.Offsets.kSpeakerToTagHeightOffset;
                
                // getting final shooter angle
                shooter_dist_to_target = Math.sqrt( 
                    Math.pow(target_pos[1], 2) + 
                    Math.pow(target_pos[2], 2)
                    );
                    
                shooter_angle = Math.acos(target_pos[1] / shooter_dist_to_target);
                
                // Rasterizing to pid values
                pid_goal = 90 - shooter_angle;
                m_shooter.setGoal(pid_goal);
            }
        }
    }
        
    @Override
    public void end(boolean interrupted){
        Input.Operator().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished(){
        return !Input.waterOnTheHill();

        // if(LimelightHelpers.getTA("") == Constants.OperatorConstants.TAG_TO_ROBOT){
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
