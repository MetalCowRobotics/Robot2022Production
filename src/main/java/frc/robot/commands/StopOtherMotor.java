package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim2;

public class StopOtherMotor extends CommandBase{
    private Cim2 m_cim;
    int count;
    public StopOtherMotor(Cim2 cim){
        m_cim = cim;
        addRequirements(m_cim);
    }

    @Override 
    public void execute(){
        // System.out.println("StopOtherMotor");
        m_cim.stop();
        count++;
    }

    @Override
    public boolean isFinished() {
        return count > 50;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop Other Motor Finished");
        count = 0;
    }
}
