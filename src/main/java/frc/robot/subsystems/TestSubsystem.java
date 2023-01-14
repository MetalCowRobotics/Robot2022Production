package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class TestSubsystem extends SubsystemBase{
    DigitalInput input = new DigitalInput(6);
    TalonSRX coolMotor = new TalonSRX(15);

    @Override
    public void periodic() {
// This method will be called once per scheduler run
        if(!input.get()){
            coolMotor.set(TalonSRXControlMode.PercentOutput, 0.3);
        }else{
            coolMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
        System.out.println(input.get());
    }

        
    }
