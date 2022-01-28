package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cim2;

public class RunOtherMotor extends CommandBase{
    private Cim2 m_cim2;
    private boolean firstTime2 = true;
    int count = 0;
    public RunOtherMotor(Cim2 cim2){
        m_cim2 = cim2;
        addRequirements(m_cim2);
    }

    @Override 
    public void execute(){
        // System.out.println("RunOtherMotor");
        m_cim2.run();
        firstTime2 = false;
        count++;
    }

    @Override
    public boolean isFinished() {
        return !firstTime2;
        //return count > 50;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Run Other Motor Finished");
        count = 0;
    }
}