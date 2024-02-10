package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelReverseCommand extends Command{

    private final FlywheelSubsystem RightFlywheel;
    private final FlywheelSubsystem LeftFlywheel;

    public FlywheelReverseCommand(FlywheelSubsystem thisRightFlywheel, FlywheelSubsystem thisLeftFlywheel) {
        RightFlywheel = thisRightFlywheel;
        LeftFlywheel = thisLeftFlywheel;
    }

    public void initialize(){
        
    }

    public void execute(){
        LeftFlywheel.FlywheelReverse();
        RightFlywheel.FlywheelReverse();
    }

    public boolean isFinished(){
        return true;
        
    }
}
