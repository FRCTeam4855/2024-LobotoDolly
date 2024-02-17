package frc.robot.subsystems;
//2 neos, 1 through bour encoder, 1 IMU

//2 limit switches, one for each end

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
  CANSparkMax armPivotOne = new CANSparkMax(9, MotorType.kBrushless);
  SparkPIDController pivotPIDController = armPivotOne.getPIDController();

  public void setPivotDirectionForward() {
    armPivotOne.set(-1);
  }

  public void setPivotDirectionBackward() {
    armPivotOne.set(1);
  }

  public void setPivotStop() {
    armPivotOne.set(0);
  }

  public void armPivotVariable(double speed) {
    armPivotOne.set(speed);
    // armPivotTwo.set(speed);
  }

  public double getPivotPostion() {
    return armPivotOne.getEncoder().getPosition();
  }

  public void resetPivotEncoderZero() {
    armPivotOne.getEncoder().setPosition(0);
  }

  public void resetPivotEncoderVariable(double value, double slotID) {
    armPivotOne.getEncoder().setPosition(value);
  }

  // Old manual control, but it resets the zero point so DO NOT USE
  // public void setPivotPositionVariable(){
  // pivotPIDController.setReference(armPivotOne.getEncoder().getPosition(),
  // CANSparkMax.ControlType.kPosition);
  // }

  // set reference encoder position manually BUT uses PID slot 2 on sparkmax
  // (static setpoints use slot 0)
  public void setPivotPositionVariable(double position) {
    pivotPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 2);
  }

  public void initPivot() {
    // PID coefficients
    armPivotOne.restoreFactoryDefaults();
    armPivotOne.restoreFactoryDefaults();
    armPivotOne.setIdleMode(IdleMode.kBrake);
    SparkAbsoluteEncoder m_pivotEncoder = armPivotOne.getAbsoluteEncoder(Type.kDutyCycle);
    double kP = 0.1; // slot 0 static setpoints
    double kP2 = 0.2; // slot 2 manual adjustment
    double kI = 0; // .0004;
    double kD = 0; // 1.2;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = .20; // slot 0 pivot speed max
    double kMaxOutput2 = .4; // slot 2 pivot speed max
    double kMinOutput = -.20;
    double kMinOutput2 = -.4;
    pivotPIDController.setFeedbackDevice(m_pivotEncoder);
    pivotPIDController.setP(kP);
    pivotPIDController.setP(kP2, 2);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    pivotPIDController.setIZone(kIz);
    pivotPIDController.setFF(kFF);
    pivotPIDController.setOutputRange(kMinOutput, kMaxOutput);
    pivotPIDController.setOutputRange(kMinOutput2, kMaxOutput2, 2);
    m_pivotEncoder.setPositionConversionFactor(360);
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