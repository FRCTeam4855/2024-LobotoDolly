package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSetpoint;
import static frc.robot.Constants.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class ArmPivot extends SubsystemBase {
  double pivotSetpoint;
  CANSparkMax m_armPivotOne = new CANSparkMax(9, MotorType.kBrushless);
  SparkPIDController pivotPIDController = m_armPivotOne.getPIDController();


  //unnecessary manual controls, not needed when using setpoint control
  
  // public void setPivotDirectionForward() {
  //   m_armPivotOne.set(-1);
  // }

  // public void setPivotDirectionBackward() {
  //   m_armPivotOne.set(1);
  // }

  // public void setPivotStop() {
  //   m_armPivotOne.set(0);
  // }

  // public void armPivotVariable(double speed) {
  //   m_armPivotOne.set(speed);
  // }

  public double getPivotPostion() {
    return m_armPivotOne.getEncoder().getPosition();
  }

  //not needed for absolute encoder
  // public void resetPivotEncoderZero() {
  //   m_armPivotOne.getEncoder().setPosition(0);
  // }



  public void initPivot() {
    // PID coefficients
    m_armPivotOne.restoreFactoryDefaults();
    m_armPivotOne.setIdleMode(IdleMode.kBrake);
    SparkAbsoluteEncoder m_pivotEncoder = m_armPivotOne.getAbsoluteEncoder(Type.kDutyCycle);
    double kP = .05; 
    double kI = 0; 
    double kD = 0; 
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    pivotPIDController.setFeedbackDevice(m_pivotEncoder);
    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    pivotPIDController.setIZone(kIz);
    pivotPIDController.setFF(kFF);
    pivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
    m_pivotEncoder.setPositionConversionFactor(360);
    m_pivotEncoder.setVelocityConversionFactor(365);
  }

  public void setPivotSetpoint(ArmSetpoint armSetpoint) {
    if (armSetpoint == ArmSetpoint.One)
      pivotSetpoint = kArmSetpoint1;
    if (armSetpoint == ArmSetpoint.Two)
      pivotSetpoint = kArmSetpoint2;
    if (armSetpoint == ArmSetpoint.Three)
      pivotSetpoint = kArmSetpoint3;
    if (armSetpoint == ArmSetpoint.Four)
      pivotSetpoint = kArmSetpoint4;
    if (armSetpoint == ArmSetpoint.Five)
      pivotSetpoint = kArmSetpoint5;
  }

  public double getPivotSetpointPosition() {
    return pivotSetpoint;
  }

  public boolean isPivotAtSetpoint() {
    return getPivotPostion() - pivotSetpoint <= kArmPivotSlop;
  }

  public void pivotDaArm() {
    // set PID coefficients
    pivotPIDController.setReference(pivotSetpoint, CANSparkMax.ControlType.kPosition);
    // SmartDashboard.putNumber("PivotSetPoint", pivotSetpoint);
    // SmartDashboard.putNumber("PivotVariable", getPivotPostion());

  }
}

// public class ArmPivot extends SubsystemBase {
//   final SparkAbsoluteEncoder m_ArmEncoder;
//   final CANSparkMax m_ArmSparkMax;
//   final SparkPIDController m_ArmPIDController; 
//   {

//   m_ArmSparkMax = new CANSparkMax(9, MotorType.kBrushless);
//   m_ArmSparkMax.setInverted(true);
//   m_ArmEncoder = m_ArmSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
//   m_ArmEncoder.setVelocityConversionFactor(365);
//   m_ArmEncoder.setPositionConversionFactor(365);
  
//   m_ArmPIDController = m_ArmSparkMax.getPIDController();
//   m_ArmPIDController.setFeedbackDevice(m_ArmEncoder);

//   m_ArmPIDController.setP(.05);
//   m_ArmPIDController.setI(0);
//   m_ArmPIDController.setD(0);
//   m_ArmPIDController.setFF(0);

//   m_ArmPIDController.setReference(20,CANSparkMax.ControlType.kPosition);
//    };  
// }