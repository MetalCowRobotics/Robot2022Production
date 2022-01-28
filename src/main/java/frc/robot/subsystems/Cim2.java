package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Cim2 extends SubsystemBase {
    private static VictorSP testMotor = new VictorSP(0);

    private double speed = 0;

    public void run(){
        speed = 0.5;
        testMotor.set(speed);
    }

    public void stop(){
        speed = 0;
        testMotor.set(speed);
    }

    @Override
    public void periodic(){
    }
}
