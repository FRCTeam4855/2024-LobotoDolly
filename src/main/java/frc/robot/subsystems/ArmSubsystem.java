package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_armSparkMax;
    //private final AbsoluteEncoder m_armThroughBoreEncoder;

    public ArmSubsystem(){
        m_armSparkMax = new CANSparkMax(15, MotorType.kBrushless);            
        };
    }
