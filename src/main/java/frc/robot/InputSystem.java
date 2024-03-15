package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class InputSystem {

    // Controllers
    private static XboxController dController = new XboxController(0);
    private static XboxController oController = new XboxController(1);
    public static XboxController Driver(){ return dController; } 
    public static XboxController Operator(){ return oController; }

    
    // 0 => Robot has no gamepiece
    // 1 -> Robot has gamepiece
    public static int input_mode = 0;
    
    // For drivetrain only
    public static double SpeedLimit(){ return 1; } 


    /* DRIVER CONTROLS */
    public static double DriveSpeed(){ return dController.getLeftY() * SpeedLimit(); } 
    public static double DriveRot(){ return dController.getRightX(); } 
    public static boolean FaceForward(){ return dController.getYButton(); } 
    public static boolean FaceDriver(){ return dController.getAButton(); } 
    public static boolean FaceRight(){  return dController.getBButton();  } 
    public static boolean FaceLeft(){ return dController.getXButton(); } 
    public static boolean Align(){ return dController.getRightBumper(); }
    public static boolean AmpAlign(){ return dController.getLeftBumper(); }
    public static double Decelerate() {return dController.getLeftTriggerAxis(); }
    
    /* OPERATOR CONTROLS */

    public static boolean SpinShooterEarly(){ return oController.getXButtonPressed(); } 
    public static boolean AutoResetAngle(){ return oController.getBButtonPressed(); } 

    // Manual Intake Controls
    public static int ManualIntake() {return oController.getPOV(180); }
    public static int ManualOuttake() {return oController.getPOV(0); }
    
    // Manual Controls for pivots
    public static double ManualIntakePivot(){ return oController.getRightY(); } 
    public static double ManualShooterPivot(){ return oController.getLeftY(); } 
    public static double ClimberController(){ return ((oController.getLeftTriggerAxis() * 1) + (oController.getRightTriggerAxis() * -1)); } 

    // Manual Climb
    public static boolean AutoIntake() {return oController.getLeftBumper();}
    public static boolean Shoot() {return oController.getRightBumper();}

    // Cancel Operator Commands  
    public static boolean AutoShutdown(){ return oController.getYButtonPressed(); } 
    
    // LED signaling for player station 
    //public static boolean AutoAmp(){ return oController.getAButtonPressed(); } for fun
    
    // Positions flipper intake to assigned positions 
    public static boolean SourcePickup(){ return oController.getLeftBumper(); } 
    public static boolean GroundPickup(){ return oController.getRightBumper(); } 
    public static boolean Retract(){ 
        return
        ( 
            ( !GroundPickup() && !SourcePickup() ) // Operator isnt requesting anything on behalf of the intake
            //|| ( Or Gamepiece detected && input_mode == 0)
        ); } 
}
