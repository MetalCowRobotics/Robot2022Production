package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
This can be an InstantCommand()
   // Deploy the Intake structure when button is pressed
   new JoystickButton(m_driverController, Button.kCircle.value)
        .whenPressed(new InstantCommand(m_intakeSubsystem::deployIntake, m_intakeSubsystem));
*/

public class HoodUp extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;

    public HoodUp(ShooterSubsystem m_shooterSubsystem) {
        this.m_shooterSubsystem = m_shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooterSubsystem.hoodUp();
        SmartDashboard.putData(m_shooterSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}