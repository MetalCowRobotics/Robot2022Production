package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbToHighBar extends SequentialCommandGroup {
    private ClimberSubsystem m_climberSubsystem;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public ClimbToHighBar(ClimberSubsystem climber, DrivetrainSubsystem drive, IntakeSubsystem intake) {
        m_climberSubsystem = climber;
        m_drivetrainSubsystem = drive;
        addRequirements(climber);
        addRequirements(drive);
        addCommands(
            new DeployIntake(intake),
            new ExtendElevatorToLength(climber, Constants.CLIMB_MID_BAR_EXTENDED_POSITION),
            new DriveStraight(180, 0.2, drive, 10),
            new ExtendElevatorToLength(climber, Constants.CLIMB_FULL_RETRACTED_POSITION),

            new ExtendElevatorToLength(climber, Constants.CLIMB_CLEAR_BAR_POSITION),
            new SendElevatorOut(climber),
            new ExtendElevatorToLength(climber, Constants.CLIMB_HIGH_BAR_EXTENDED_POSITION),
            new PullElevatorIn(climber),
            new ExtendElevatorToLength(climber, Constants.CLIMB_FULL_RETRACTED_POSITION)
        );
    }

    @Override
    public void execute() {
        // isFinished = true;
    }
}
