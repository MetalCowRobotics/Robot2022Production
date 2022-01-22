package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim;

public class StopMotor extends CommandBase{
    private Cim m_cim;
    int count;

    public StopMotor(Cim cim){
        m_cim = cim;
        addRequirements(m_cim);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("SM Count: " + count>50);
        return count > 50;
    }

    @Override 
    public void execute(){
        //System.out.println("StopMotor");
        m_cim.stop();
        count++;
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop Motor Finished");
        count = 0;
    }
}