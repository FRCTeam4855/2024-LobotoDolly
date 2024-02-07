package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInputCommand extends Command{

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
