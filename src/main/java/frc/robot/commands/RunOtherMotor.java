package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim2;

public class RunOtherMotor extends CommandBase{
    private Cim2 m_cim;
    int count = 0;
    public RunOtherMotor(Cim2 cim){
        m_cim = cim;
        addRequirements(m_cim);
    }

    @Override 
    public void execute(){
        // System.out.println("RunOtherMotor");
        m_cim.run();
        count++;
    }

    @Override
    public boolean isFinished() {
        return count > 50;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Run Other Motor Finished");
        count = 0;
    }
}