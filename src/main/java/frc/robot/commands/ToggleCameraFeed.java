package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class ToggleCameraFeed extends SequentialCommandGroup {
    int cameraFeed;

    public ToggleCameraFeed() {
        
    }

    @Override
    public void execute() {
        //0 -> __ Camera
        //1 -> __ Camera
        if (cameraFeed == 0) {
            cameraFeed = 1;
        }
        else {
            cameraFeed = 0;
        }
    }

}
