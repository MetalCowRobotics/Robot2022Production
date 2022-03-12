package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootBall extends SequentialCommandGroup{

    public ShootBall(ShooterSubsystem shooter, MagazineSubsystem magazine){
        addCommands(

        new ShootSpeedMagazine (shooter, magazine),

        new StopShooterWheel(shooter)
            
        );
    
    }

    

    
}