package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Cim extends SubsystemBase {
    private static VictorSP testMotor = new VictorSP(1);

    private double speed = 0;

    public void run(){
        speed = 0.5;
    }

    public void stop(){
        speed = 0;
    }

    @Override
    public void periodic(){
        testMotor.set(speed);
    }
}
