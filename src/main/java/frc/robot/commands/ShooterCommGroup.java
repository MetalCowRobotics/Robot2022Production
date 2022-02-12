package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.ShooterSubSystem;

public class ShooterCommGroup extends SequentialCommandGroup{

    public ShooterCommGroup(Magazine mag, ShooterSubSystem shooter) {
        addCommands(
            




        );
    }

    

    
}
