package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax s_neo = new CANSparkMax(10, MotorType.kBrushless);
    
    public double speed = 0;
    public final double wantedSpeed = 0.1;

    public void run(){
        speed = 0.1;
    }

    public void stop(){
        speed = 0;
    }

    public boolean isReady(){
        if (s_neo.get() >= wantedSpeed){
            return true;        
        } else {
            return false;
        }
    }

    private static final Shooter instance = new Shooter();
    public static Shooter getInstance() {
        return instance;
    }

    @Override
    public void periodic(){
        s_neo.set(speed);
        SmartDashboard.putNumber("Speed", s_neo.get());
        SmartDashboard.putBoolean("isReady", isReady());
        SmartDashboard.putNumber("WantedSpeed", wantedSpeed);
    }
}
