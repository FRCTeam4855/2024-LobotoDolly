package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelSpeakerCommand extends Command{

    private final FlywheelSubsystem RightFlywheel;
    private final FlywheelSubsystem LeftFlywheel;

    public FlywheelSpeakerCommand(FlywheelSubsystem thisRightFlywheel, FlywheelSubsystem thisLeftFlywheel) {
        RightFlywheel = thisRightFlywheel;
        LeftFlywheel = thisLeftFlywheel;
    }

    public void initialize(){
        
    }

    public void execute(){ //5.7 degrees
        LeftFlywheel.FlywheelSpeaker();
        RightFlywheel.FlywheelSpeaker();
    }

    public boolean isFinished(){
        return true;
        
    }
}