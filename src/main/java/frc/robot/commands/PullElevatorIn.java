package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class PullElevatorIn extends CommandBase{
    private ClimberSubsystem m_climberSubsystem;
    private boolean isFinished = false;

    public PullElevatorIn(ClimberSubsystem climber) {
        m_climberSubsystem = climber;
    }

    @Override
    public void execute() {
        m_climberSubsystem.retractClimber();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
