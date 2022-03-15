package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoadBall extends CommandBase {

    private MagazineSubsystem m_magazineSubsystem;
    private IntakeSubsystem m_intakeSubsystem;

    public LoadBall(MagazineSubsystem magazine, IntakeSubsystem intake) {
        m_magazineSubsystem = magazine;
        m_intakeSubsystem = intake;
    }

    public void initialize() {
        new StopGathering(m_intakeSubsystem);
    }

    public void periodic() {
        new StartMagazine(m_magazineSubsystem);
    }

    public void end() {
        new StopMagazine(m_magazineSubsystem);
    }

    

    
}