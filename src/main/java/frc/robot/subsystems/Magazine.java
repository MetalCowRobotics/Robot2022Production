package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Magazine extends SubsystemBase {
    private static Spark magMotor = new Spark(0);

    private double speed = 0;

    public void run(){
        speed = 0.5;
    }

    public void stop(){
        speed = 0;
    }

    @Override
    public void periodic(){
        magMotor.set(speed);
    }
}
