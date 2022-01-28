package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim2;

public class StopOtherMotor extends CommandBase{
    private Cim2 m_cim2;
    int count;
    public StopOtherMotor(Cim2 cim2){
        m_cim2 = cim2;
        addRequirements(m_cim2);
    }

    @Override 
    public void execute(){
        // System.out.println("StopOtherMotor");
        m_cim2.stop();
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
