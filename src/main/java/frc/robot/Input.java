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

    public static double lowerLeftClimb(){
        return driveController.getLeftTriggerAxis();
    }

    public static double lowerRightClimb(){
        return driveController.getRightTriggerAxis();
    }

    /* SHOOTER CONTROLS */
    public static double fireInTheHole(){ // This is to shoot... not intake || TODO: Get this to be shoot instead
        return (opController.getRightTriggerAxis());
    }

    public static boolean AmpScore(){
        return (opController.getLeftBumper());
    }

    public static double rockOnTheGround(){ // Pivot Control :sob:
        return (opController.getRightY() * 0.3);
    }

    public static boolean waterOnTheHill(){  // This is to intake... not shoot || TODO: Get this to be intake instead
        return (opController.getRightBumper());
    }

    /* DRIVER AND OPERATOR */
    public static XboxController Driver(){
        return driveController;
    }

    public static XboxController Operator(){
        return opController;
    }

    public static boolean AutoShutdown(){
        return driveController.getStartButton() || opController.getStartButton();
    }

    // public static boolean IntakeGamepiece(){
    //     return opController.getRightBumper();
    // }
}
