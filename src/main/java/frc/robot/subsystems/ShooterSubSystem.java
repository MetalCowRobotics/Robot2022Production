package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShooterSubSystem extends SubsystemBase{
    private CANSparkMax shootMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless);
    private final double wantedSpeed = 0.2;

    private double speed = 0;

    public ShooterSubSystem() {
        
    }

    public void run() {
        speed = 0.2;
    }

    public void stop() {
        speed = 0;
    }

    @Override
    
    public void periodic() {
        shootMotor.set(speed);
        SmartDashboard.putNumber("Speed", shootMotor.get());
        SmartDashboard.putBoolean("isReady", isReady());
        SmartDashboard.putNumber("WantedSpeed", wantedSpeed);
    }

    public boolean isReady(){
        if (shootMotor.get() >= wantedSpeed){
            return true;        
        } else {
            return false;
        }
    }

}
