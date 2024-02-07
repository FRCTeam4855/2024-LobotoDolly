package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheelSubsystem extends SubsystemBase {
    private final CANSparkMax m_rightFlywheelSparkMax;
    private final CANSparkMax m_leftFlywheelSparkMax;
    public FlyWheelSubsystem() {
        m_rightFlywheelSparkMax = new CANSparkMax(14, MotorType.kBrushless);
        m_leftFlywheelSparkMax = new CANSparkMax(15, MotorType.kBrushless);
    }
    public void FlywheelSpeaker() {
        m_rightFlywheelSparkMax.set(1);
        m_leftFlywheelSparkMax.set(1);
    }
    public void FlywheelAmp() {
        m_rightFlywheelSparkMax.set(.1);
        m_leftFlywheelSparkMax.set(.1);
    }
    public void FlywheelReverse() {
        m_rightFlywheelSparkMax.set(-.5);
        m_leftFlywheelSparkMax.set(-.5);
    }
}