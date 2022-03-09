package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendElevatorToLength extends CommandBase{
    private ClimberSubsystem m_climberSubsystem;
    private boolean isFinished = false;
    private double target;

    public ExtendElevatorToLength(ClimberSubsystem climber, double ticsTarget) {
        m_climberSubsystem = climber;
        target = ticsTarget;
    }

    @Override
    public void execute() {
        if (m_climberSubsystem.getElevatorPosition() > target) {
            m_climberSubsystem.retractClimberMotor();
        } else {
            m_climberSubsystem.extendClimberMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(target - m_climberSubsystem.getElevatorPosition()) <= ClimberSubsystem.ELEVATOR_MOTOR_DEADZONE;
    }
}
