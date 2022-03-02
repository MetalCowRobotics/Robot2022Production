package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooting extends SequentialCommandGroup {

    public StartShooting(MagazineSubsystem mag) {
        // add command to put drivetrain on max brake
        addCommands(new StartMagazine(mag));
    }

    @Override
    public void execute() {
    }

}
