package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoint;
import frc.robot.Subsystems.ArmExtend;
import frc.robot.Subsystems.ArmPivot;

public class MoveArmToSetpoint extends CommandBase {

  private ArmExtend armExtend;
  private ArmPivot armPivot;
  private double startExtendEncoderSetpoint;
  private double goalExtendEncoderSetpoint;
  private double startPivotEncoderSetpoint;
  private double goalPivotEncoderSetpoint;
  private ArmSetpoint startArmSetpoint;
  private ArmSetpoint goalArmSetpoint;
  private double startTime;

  public MoveArmToSetpoint(ArmExtend armExtend, ArmPivot armPivot, ArmSetpoint goalArmSetpoint,
      ArmSetpoint startArmSetpoint) {
    this.armExtend = armExtend;
    this.armPivot = armPivot;
    this.startArmSetpoint = startArmSetpoint;
    this.goalArmSetpoint = goalArmSetpoint;
    addRequirements(armExtend, armPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startExtendEncoderSetpoint = armExtend.getExtensionPostion();
    // goalExtendEncoderSetpoint = armExtend.getExtendSetpointPosition();
    // startPivotEncoderSetpoint = armPivot.getPivotPostion();
    // goalPivotEncoderSetpoint = armPivot.getPivotSetpointPosition();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goalArmSetpoint == ArmSetpoint.One) {
      armExtend.setExtendSetpoint(goalArmSetpoint);
      armExtend.extendDaArm();
      if (Timer.getFPGATimestamp() - startTime > .4) {
        armPivot.setPivotSetpoint(goalArmSetpoint);
        armPivot.pivotDaArm();
      }
    }

    if (goalArmSetpoint != ArmSetpoint.One) {
      armPivot.setPivotSetpoint(goalArmSetpoint);
      armPivot.pivotDaArm();
      if (Timer.getFPGATimestamp() - startTime > .4) {
        armExtend.setExtendSetpoint(goalArmSetpoint);
        armExtend.extendDaArm();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > .4) {
      return true;
    } else {
      return false;
    }
  }
}