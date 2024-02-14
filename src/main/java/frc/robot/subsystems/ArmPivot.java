package frc.robot.subsystems;
import com.revrobotics.SparkAbsoluteEncoder.Type;

//import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
//import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSetpoint;
import static frc.robot.Constants.*;
//import com.revrobotics.SparkAbsoluteEncoder.Type;


public class ArmPivot extends SubsystemBase {
  final SparkAbsoluteEncoder m_ArmEncoder;
  final CANSparkMax m_ArmSparkMax;
  final SparkPIDController m_ArmPIDController; 
  {

  m_ArmSparkMax = new CANSparkMax(9, MotorType.kBrushless);
  m_ArmSparkMax.setInverted(true);
  m_ArmEncoder = m_ArmSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
  m_ArmEncoder.setVelocityConversionFactor(365);
  m_ArmEncoder.setPositionConversionFactor(365);
  
  m_ArmPIDController = m_ArmSparkMax.getPIDController();
  m_ArmPIDController.setFeedbackDevice(m_ArmEncoder);

  m_ArmPIDController.setP(.05);
  m_ArmPIDController.setI(0);
  m_ArmPIDController.setD(0);
  m_ArmPIDController.setFF(0);

  m_ArmPIDController.setReference(20,CANSparkMax.ControlType.kPosition);
   };  
}
