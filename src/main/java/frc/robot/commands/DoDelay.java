package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoDelay extends CommandBase {

    private double timeout;
    private Timer timer = new Timer();

    public DoDelay(double seconds) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        timeout = seconds;
    }

    protected void startTimer() {
        startTimer();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return timer.get() > timeout;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

}
