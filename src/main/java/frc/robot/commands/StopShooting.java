package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooting extends SequentialCommandGroup {

    public StopShooting(MagazineSubsystem mag) {
        // add command to put drivetrain on max brake
        addCommands(new StopMagazine(mag));
    }

    @Override
    public void execute() {
    }

}
