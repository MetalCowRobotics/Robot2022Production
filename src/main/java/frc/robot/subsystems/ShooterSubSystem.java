package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShooterSubSystem extends SubsystemBase{
    private CANSparkMax shootMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless);
    

    private double speed = 0;

    public void run() {
        speed = 0.2;
    }

    public void stop() {
        speed = 0;
    }

    @Override
    
    public void periodic() {
        shootMotor.set(speed);
    }

}
