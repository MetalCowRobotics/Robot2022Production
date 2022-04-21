package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(MagazineSubsystem m_magazineSubsystem, ShooterSubsystem m_ShooterSubsystem, DrivetrainSubsystem m_drivetrainSubsystem) {
        m_ShooterSubsystem.hoodCloseShot();
        addCommands(
            new StartShooterWheel(m_ShooterSubsystem), 
            new DoDelay(3), 
            new StartMagazine(m_magazineSubsystem), 
            new DoDelay(2), 
            new DriveStraight(270, 0.3, m_drivetrainSubsystem, 90)
        );
    }
    
}
