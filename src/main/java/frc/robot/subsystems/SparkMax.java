package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SparkMax extends SubsystemBase {
    private static CANSparkMax sparkMax;

    private double speed = 0;

    public SparkMax (int ID) {
        sparkMax = new CANSparkMax(ID, MotorType.kBrushless);
    }

    public void run(){
        speed = 0.2;
    }

    public void stop(){
        speed = 0;
    }

    @Override
    public void periodic(){
        sparkMax.set(speed);
    }
}