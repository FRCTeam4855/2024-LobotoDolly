package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    public final CANSparkMax m_rightFlywheelSparkMax;
    public final CANSparkMax m_leftFlywheelSparkMax;
    public FlywheelSubsystem() {
        m_rightFlywheelSparkMax = new CANSparkMax(12, MotorType.kBrushless);
        m_leftFlywheelSparkMax = new CANSparkMax(11, MotorType.kBrushless);
    }
    public void FlywheelSpeaker() {
        m_rightFlywheelSparkMax.set(.4);
        m_leftFlywheelSparkMax.set(.5);
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