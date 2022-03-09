package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ClimbToHighBar extends SequentialCommandGroup {
    private ClimberSubsystem m_climberSubsystem;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public ClimbToHighBar(ClimberSubsystem climber, DrivetrainSubsystem drive) {
        m_climberSubsystem = climber;
        m_drivetrainSubsystem = drive;
        addCommands(
            new SendElevatorOut(climber),
            new 
            new DriveStraight(180, 0.2, drive, 10),
            new PullElevatorIn(climber),
            new 
        )
    }

    @Override
    public void execute() {
        m_climberSubsystem.retractClimberMotor();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
