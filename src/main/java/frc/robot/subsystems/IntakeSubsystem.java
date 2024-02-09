package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MAXSwerveModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeSubsystem extends SubsystemBase {
    
    
    private final CANSparkMax m_intakeSparkMax;
    private final SparkPIDController m_intakePIDController;
    public ColorSensorV3 noteSensor;
    //int proximity = noteSensor.getProximity();
    public IntakeSubsystem() {
    m_intakeSparkMax = new CANSparkMax(10, MotorType.kBrushless);
    m_intakePIDController = m_intakeSparkMax.getPIDController();
    noteSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }
    public void NoteInIntake() {
        if (noteSensor.getProximity() == 1000){
            m_intakeSparkMax.set(1);
        }
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