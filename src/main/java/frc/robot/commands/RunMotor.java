package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim;

public class RunMotor extends CommandBase{
    private Cim m_cim;
    private boolean firstTime = true;
    int count;
    public RunMotor(Cim cim){
        m_cim = cim;
        addRequirements(m_cim);
    }

    @Override 
    public void execute() {
        // System.out.println("RunMotor");
        m_cim.run();
        firstTime = false;
        count++;
    }

    @Override
    public boolean isFinished() {
        return !firstTime;
        // return count > 50;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Run Motor Finished");
        count = 0;
    }
}