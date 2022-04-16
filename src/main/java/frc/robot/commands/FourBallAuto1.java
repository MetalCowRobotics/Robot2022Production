package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAuto1 extends SequentialCommandGroup {
    public FourBallAuto1(IntakeSubsystem m_intakeSubsystem, DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_ShooterSubsystem, MagazineSubsystem m_magazineSubsystem, VisionSubsystem m_visionSubsystem) {
        addCommands(
            new StartShooterWheel(m_ShooterSubsystem),
            new HoodUp(m_ShooterSubsystem),
            new StartGathering(m_intakeSubsystem),
            // new StartGathering(m_intakeSubsystem),
            new DriveStraight(90, 0.60, m_drivetrainSubsystem, 43 + 21),
            // new DoDelay(0.5),
            // new StopGathering(m_intakeSubsystem),
            new TurnDegrees(m_drivetrainSubsystem, 190, 1),
            new VisionTrackingAuto(m_drivetrainSubsystem, m_visionSubsystem, 0.5),
            // new DriveStraight(90, 0.45, m_drivetrainSubsystem, 20),
            new DoDelay(0.2),
            new StartIntakeWheels(m_intakeSubsystem),
            new StartMagazine(m_magazineSubsystem),
            new DoDelay(1),
            new StopIntakeWheels(m_intakeSubsystem),
            new StopMagazine(m_magazineSubsystem),
            new DoDelay(0.1),
            new StartIntakeWheels(m_intakeSubsystem),
            new TurnDegrees(m_drivetrainSubsystem, 0, 1),
            new DoDelay(0.1),
            new DriveStraight(63, 0.75, m_drivetrainSubsystem, 136),
            new DoDelay(1.75),
            new DriveStraight(-104, 0.75, m_drivetrainSubsystem, 124),
            new TurnDegrees(m_drivetrainSubsystem, 190
            , -1),
            new DoDelay(0.1),
            // new StartIntakeWheels(m_intakeSubsystem),
            new VisionTrackingAuto(m_drivetrainSubsystem, m_visionSubsystem, .75),
            new StartMagazine(m_magazineSubsystem)
        );

        
    }
}
