package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSystem;

public class ShooterStop extends CommandBase {
    private ShooterSubSystem m_motor1;
    private boolean checkState = true;
    public ShooterStop(ShooterSubSystem motor1) {
       m_motor1 = motor1;
       addRequirements(m_motor1);
    }

    @Override 
    public void execute() {
        m_motor1.stop();
        checkState = false;
    }

    @Override
    public boolean isFinished() {
        return !checkState;
    }

    @Override
    public void end(boolean interrupted) {   
    }
}
