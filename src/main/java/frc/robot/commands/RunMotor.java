package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim;

public class RunMotor extends CommandBase{
    private Cim m_cim;
    int count;
    public RunMotor(Cim cim){
        m_cim = cim;
        addRequirements(m_cim);
    }

    @Override 
    public void execute() {
        // System.out.println("RunMotor");
        m_cim.run();
        count++;
    }

    @Override
    public boolean isFinished() {
        return count > 50;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ru Motor Finished");
        count = 0;
    }
}