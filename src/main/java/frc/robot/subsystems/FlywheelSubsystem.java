package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    CANSparkMax m_rightFlywheelSparkMax = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax m_leftFlywheelSparkMax = new CANSparkMax(11, MotorType.kBrushless); // right .4 and left .5 for speaker
    public boolean FlywheelOn;
    public void FlywheelVariable(double speed) {
        m_rightFlywheelSparkMax.set(speed);
        m_leftFlywheelSparkMax.set(speed);
    }

    public void FlywheelStop() {
        m_rightFlywheelSparkMax.set(0);
        m_leftFlywheelSparkMax.set(0);
    }

}