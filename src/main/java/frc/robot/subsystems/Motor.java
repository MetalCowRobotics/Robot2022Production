package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public class Motor extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(18, MotorType.kBrushless);
    private double speed = 0;
    public void runMotor(double speedMetersPerSecond) {
        speed = speedMetersPerSecond / 18.0;
        speed = Math.min(speed, 0.5);
    }

    @Override 
    public void periodic() {
        motor.set(speed);
    }
}