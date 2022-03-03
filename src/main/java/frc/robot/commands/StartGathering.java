package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class StartGathering extends SequentialCommandGroup{
    public StartGathering(IntakeSubsystem intake) {
        addCommands(
            new DeployIntake(intake),
            new StartIntakeWheels(intake)
        );
    }
}
