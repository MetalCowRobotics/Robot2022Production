package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FourBallAuto1 extends SequentialCommandGroup {
    public FourBallAuto1(IntakeSubsystem m_intakeSubsystem, DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_ShooterSubsystem, MagazineSubsystem m_magazineSubsystem) {
        addCommands(
            new HoodUp(m_ShooterSubsystem),
            new StartGathering(m_intakeSubsystem),
            new StartShooterWheel(m_ShooterSubsystem),
            // new StartGathering(m_intakeSubsystem),
            new DriveStraight(90, 0.45, m_drivetrainSubsystem, 115),
            // new DoDelay(0.5),
            // new StopGathering(m_intakeSubsystem),
            new TurnDegrees(m_drivetrainSubsystem, 195, -1),
            // new DriveStraight(90, 0.45, m_drivetrainSubsystem, 20),
            new DoDelay(1),
            new StartIntakeWheels(m_intakeSubsystem),
            new StartMagazine(m_magazineSubsystem),
            new DoDelay(1),
            new StopIntakeWheels(m_intakeSubsystem),
            new StopMagazine(m_magazineSubsystem),
            new TurnDegrees(m_drivetrainSubsystem, 191, 1),
            new DriveStraight(79, 0.45, m_drivetrainSubsystem, 160),
            new DoDelay(2),
            new DriveStraight(-101, 0.45, m_drivetrainSubsystem, 160),
            new TurnDegrees(m_drivetrainSubsystem, 191, -1),
            new StartIntakeWheels(m_intakeSubsystem),
            new StartMagazine(m_magazineSubsystem)
        );

        
    }
}
