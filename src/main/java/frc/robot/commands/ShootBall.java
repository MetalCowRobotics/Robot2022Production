package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootBall extends SequentialCommandGroup{

    public ShootBall(ShooterSubSystem shooter, DrivetrainSubsystem drive, double Delay){
        addCommands(
            new DoDelay(Delay),

            new DriveToCoordinate(drive, SmartDashboard.getNumber("x Amount", 0.5), SmartDashboard.getNumber("y Amount", 0.5))
        );
    
    }

    

    
}