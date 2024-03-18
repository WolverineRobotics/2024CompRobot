package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.InputSystem;
import frc.robot.commands.DefaultNoahCommand;

public class NoahDriveSubsystem extends SubsystemBase{
    // Declare Motors Here
    private CANSparkMax _leftMaster;
    private CANSparkMax _rightMaster; 
    private CANSparkMax _leftFollower;
    private CANSparkMax _rightFollower;

    // 
    private RelativeEncoder _leftEncoder;
    private RelativeEncoder _rightEncoder;

    private DifferentialDrive _driveTrain;
    private DefaultNoahCommand defaultCommand;
    
    public NoahDriveSubsystem() {
        _leftMaster = new CANSparkMax(Constants.LEFT_MOTOR_1, MotorType.kBrushless);
        _rightMaster = new CANSparkMax(Constants.RIGHT_MOTOR_1, MotorType.kBrushless);
        _leftFollower = new CANSparkMax(Constants.LEFT_MOTOR_2, MotorType.kBrushless);
        _rightFollower = new CANSparkMax(Constants.RIGHT_MOTOR_2, MotorType.kBrushless);
        
        _leftFollower.follow(_leftMaster, false);
        _rightFollower.follow(_rightMaster, false);
        
        
        _leftEncoder = _leftMaster.getEncoder();
        _rightEncoder = _rightMaster.getEncoder();
        
        _driveTrain = new DifferentialDrive(_leftMaster, _rightMaster);
        
        //defaultCommand = new DefaultNoahCommand(this);
        //setDefaultCommand(defaultCommand);
    }
    /*@Override
    public void periodic() {
        // _driveTrain.arcadeDrive(InputSystem.DriveSpeed(), InputSystem.DriveRot());
    }*/
    public DifferentialDrive getDriveTrain() {
        return _driveTrain;
    }

}