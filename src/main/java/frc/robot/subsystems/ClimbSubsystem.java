package frc.robot.subsystems;
import java.util.Set;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Input;

public class ClimbSubsystem extends SubsystemBase  {

    public final CANSparkMax leftClimb;
    public final CANSparkMax rightClimb;

    private int left_timer = 0;
    private int right_timer = 0;

    public ClimbSubsystem(){
        leftClimb = new CANSparkMax(4, MotorType.kBrushless);
        rightClimb = new CANSparkMax(9, MotorType.kBrushless);

        leftClimb.setIdleMode(IdleMode.kBrake);
        rightClimb.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic(){
    }
}
