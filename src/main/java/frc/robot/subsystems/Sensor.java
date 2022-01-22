package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensor extends SubsystemBase {
    DigitalInput GARAGE_SENSOR = new DigitalInput(0);



    @Override
    public void periodic() {

    }
    public boolean objectInFront() {
        return GARAGE_SENSOR.get();
    }

    public boolean objectNotInFront() {
        return !GARAGE_SENSOR.get();
    }
}
