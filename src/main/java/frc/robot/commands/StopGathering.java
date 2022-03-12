package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class StopGathering extends SequentialCommandGroup{
    public StopGathering(IntakeSubsystem intake) {
        addCommands(
            new StopIntakeWheels(intake),
            new RetractIntake(intake)        
        );
    }
}
