package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterWheel extends CommandBase {
    private ShooterSubsystem m_motor1;
    private boolean checkState = true;
    public SpinUpShooterWheel(ShooterSubsystem motor1) {
       m_motor1 = motor1;
       addRequirements(m_motor1);
    }

    @Override 
    public void execute() {
        m_motor1.shootLow();
        checkState = false;
    }

    @Override
    public boolean isFinished() {
        return m_motor1.isReady();
    }

    @Override
    public void end(boolean interrupted) {   
    }
}
