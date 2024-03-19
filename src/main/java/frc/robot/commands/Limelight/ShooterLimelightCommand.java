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
import frc.robot.Constants;
import frc.robot.Input;

public class ShooterLimelightCommand extends Command{

    // TODO: Actually get an ID, not an ID of 0...
    private double target = 0; 
    private ShooterSubsystem m_Shooter;
    private XboxController dController;

    private LimelightSubsystem m_Limelight;
    
    private Double[] targetPosition;
    private Double shooterDistToTarget, shooterAngle, pidGoal;
    private Pose3d targetPose;

    public boolean m_AprilTaginSight = false, atGoal;

    public ShooterLimelightCommand(ShooterSubsystem shooter, LimelightSubsystem limelight){

        m_Shooter = shooter;
        m_Limelight = limelight;
        addRequirements(shooter, limelight);
    }

    /* TRACK THE TARGET */
    @Override
    public void execute(){
        // if(LimelightHelpers.getTV("")){
        //     if(LimelightHelpers.getFiducialID("") == target){
        //         // If limelight sees shooter target
                
        //         // Setting indication and rumble
        //         if(m_Shooter.getController().atGoal() && !atGoal){
        //             Input.Operator().setRumble(RumbleType.kBothRumble, 0.3);
        //         }
        //         else{
        //             atGoal = false;
        //             Input.Operator().setRumble(RumbleType.kBothRumble, 0);
        //         }
                
        //         // Distances are in metres
        //         // Getting initial limelights target pose
        //         targetPose = LimelightHelpers.getTargetPose3d_CameraSpace("");
        //         targetPosition = new Double[]{
        //             targetPose.getX(),
        //             targetPose.getY(),
        //             targetPose.getZ(),
        //         };

        //         // applying offsets for accurate target pose
        //         targetPosition[2] -= Constants.Offsets.kLimelightShooterOffset;
        //         targetPosition[2] += Constants.Offsets.kSpeakerToTagHeightOffset;
                
        //         // getting final shooter angle
        //         shooterDistToTarget = Math.sqrt( 
        //             Math.pow(targetPosition[1], 2) + 
        //             Math.pow(targetPosition[2], 2)
        //             );
                    
        //         shooterAngle = Math.acos(targetPosition[1] / shooterDistToTarget);
                
        //         // Rasterizing to pid values
        //         pidGoal = 90 - shooterAngle;
        //         m_Shooter.setGoal(pidGoal);
        //     }
        // }
    }
        
    @Override
    public void end(boolean interrupted){
        Input.Operator().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished(){
        // return !Input.waterOnTheHill();

        // if(LimelightHelpers.getTA("") == Constants.OperatorConstants.TAG_TO_ROBOT){
        //     return true;
        // } else {
        //     return false;
        // }

        return false;
    }
}
