package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_intakeSparkMax;
    private final SparkPIDController m_intakePIDController;

    public IntakeSubsystem() {
    m_intakeSparkMax = new CANSparkMax(13, MotorType.kBrushless);
    m_intakePIDController = m_intakeSparkMax.getPIDController();
    }
    public void IntakeInput() {
        m_intakeSparkMax.set(1);
    }
    public void IntakeOutput() {
        m_intakeSparkMax.set(-1);
    }
    public void IntakeStop() {
        m_intakeSparkMax.set(0);
    }
}