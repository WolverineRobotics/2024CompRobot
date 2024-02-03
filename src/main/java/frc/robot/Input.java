package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

public class Input {
    
    public static XboxController driveController = new XboxController(0);
    public static XboxController OpController = new XboxController(0);

    public static double getVertical(){
        return (driveController.getLeftY());
    }

    public static double getHorizontal(){
        return driveController.getRightX() * 0.8;
    }

    public static double driveDecelerate(){
        return driveController.getLeftTriggerAxis();
        }

    public static XboxController Driver(){
        return driveController;
        }
}
