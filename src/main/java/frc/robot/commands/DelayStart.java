package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSystem;

public class DelayStart extends CommandBase {
    private double delay = 0;
    private DoDelay delayexec;
    private boolean checkState = true;
    public DelayStart(DoDelay Delay) {
        delayexec = Delay;
        SmartDashboard.putNumber("Delay", delay);
        delayexec.execute();
    }

    @Override 
    public void execute() {
        new DoDelay(delay);
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

