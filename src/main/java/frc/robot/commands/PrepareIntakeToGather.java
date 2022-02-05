package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/*
This can be an InstantCommand()
   // Deploy the Intake structure when button is pressed
   new JoystickButton(m_driverController, Button.kCircle.value)
        .whenPressed(new InstantCommand(m_intakeSubsystem::deployIntake, m_intakeSubsystem));
*/
public class PrepareIntakeToGather extends CommandBase {
    private final IntakeSubsystem m_intakeSubsytem;

    public PrepareIntakeToGather(IntakeSubsystem m_intakeSubsytem) {
        this.m_intakeSubsytem = m_intakeSubsytem;
        addRequirements(m_intakeSubsytem);
    }

    @Override
    public void initialize() {
        m_intakeSubsytem.deployIntake();
        SmartDashboard.putData(m_intakeSubsytem);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
