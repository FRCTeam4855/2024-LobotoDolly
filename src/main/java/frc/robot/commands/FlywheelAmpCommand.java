package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelAmpCommand extends Command{

    private final FlywheelSubsystem RightFlywheel;
    private final FlywheelSubsystem LeftFlywheel;

    public FlywheelAmpCommand(FlywheelSubsystem thisRightFlywheel, FlywheelSubsystem thisLeftFlywheel) {
        RightFlywheel = thisRightFlywheel;
        LeftFlywheel = thisLeftFlywheel;
    }

    public void initialize(){
        
    }

    public void execute(){
        LeftFlywheel.FlywheelAmp();
        RightFlywheel.FlywheelAmp();
    }

    public boolean isFinished(){
        return true;
        
    }
}
