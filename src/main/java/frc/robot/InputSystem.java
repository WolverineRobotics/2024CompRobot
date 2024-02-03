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
    public static boolean Balance(){ return dController.getRightBumper(); }
    public static boolean Align(){ return dController.getRightBumper(); }
    
    
    /* OPERATOR CONTROLS */

    public static boolean SpinShooterEarly(){ return oController.getXButtonPressed(); } 
    public static boolean UndefinedBind2(){ return oController.getBButtonPressed(); } 
    
    // Manual Controls for pivots
    public static double ManualIntakePivot(){ return oController.getLeftY(); } 
    public static double ManualShooterPivot(){ return oController.getRightY(); } 
    public static double RollerControl(){ return ((oController.getLeftTriggerAxis() * 1) + (oController.getRightTriggerAxis() * -1)); } 
    
    // LED signaling for player station 
    public static boolean SignalSourcePickup(){ return oController.getYButtonPressed(); } 
    public static boolean SignalGroundPickup(){ return oController.getAButtonPressed(); } 
    
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
