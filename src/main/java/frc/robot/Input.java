package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

public class Input {
    
    public static XboxController driveController = new XboxController(0);
    public static XboxController opController = new XboxController(1);

    /* DRIVE CONTROLS */
    public static double getVertical(){
        return (driveController.getLeftY());
    }

    public static double getHorizontal(){
        return driveController.getRightX() * 0.8;
    }

    public static double driveDecelerate(){
        return driveController.getLeftTriggerAxis();
    }

    public static boolean alignTag(){
        return driveController.getRightBumper();
    }

    /* SHOOTER CONTROLS */
    public static double fireInTheHole(){
        return (opController.getRightTriggerAxis());
    }

    public static boolean waterOnTheHill(){
        return (opController.getRightBumper());
    }

    /* DRIVER AND OPERATOR */
    public static XboxController Driver(){
        return driveController;
    }

    public static XboxController Operator(){
        return opController;
    }
}
