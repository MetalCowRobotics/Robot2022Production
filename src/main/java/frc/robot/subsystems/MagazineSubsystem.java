package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MagazineSubsystem extends SubsystemBase {
    private CANSparkMax magMotor = new CANSparkMax(Constants.MAGAZINE_MOTOR, MotorType.kBrushless);
    private double speed = 0;

    public void run(){
        speed = 0.05;
    }

    public void stop(){
        speed = 0;
    }

    @Override
    public void periodic(){
        magMotor.set(speed);
    }
}