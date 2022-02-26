package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterCommGroup extends SequentialCommandGroup{

    public ShooterCommGroup(Magazine mag, ShooterSubSystem shooter, DrivetrainSubsystem drive){
        addCommands(
            new DoDelay(5),

            new DriveToCoordinate(drive, SmartDashboard.getNumber("x Amount", 0.5), SmartDashboard.getNumber("y Amount", 0.5))
        );
    
    }

    

    
}
