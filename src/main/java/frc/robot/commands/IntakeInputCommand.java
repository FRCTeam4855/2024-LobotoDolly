package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInputCommand {

private final IntakeSubsystem Intake;

    public IntakeInputCommand(IntakeSubsystem thisIntake){
        Intake = thisIntake;
    }
    public void initialize() {}

    public void execute() {
        Intake.IntakeInput();
    }

    public boolean isFinished() {
        return true;
      }
}
