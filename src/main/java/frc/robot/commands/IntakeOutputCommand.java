package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutputCommand extends Command {
    private final IntakeSubsystem Intake;

    public IntakeOutputCommand(IntakeSubsystem thisIntake) {
        Intake = thisIntake;
    }

    public void initialize() {
        
    }

    public void execute() {
        Intake.IntakeRun();
    }

    public boolean isFinished() {
        if (Intake.IntakeSpeed > -.5) {
            return true;
        } else {
            return false;
        }
    }
}