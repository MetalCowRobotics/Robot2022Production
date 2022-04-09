package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;

public class MagazineSubsystem extends SubsystemBase {
    private CANSparkMax magMotor = new CANSparkMax(Constants.MAGAZINE_MOTOR, MotorType.kBrushless);
    private DigitalInput frontSensor = new DigitalInput(Constants.MAGAZINE_SENSOR_FRONT);
    private DigitalInput topSensor = new DigitalInput(Constants.MAGAZINE_SENSOR_TOP);

    private double speed = 0;
    private boolean runContinuous = false;
    private double rampRate = 0.5;
    public MagazineSubsystem() {
        magMotor.setOpenLoopRampRate(rampRate);
        magMotor.setClosedLoopRampRate(rampRate);
    }

    public boolean getBallInFront() {
        return !frontSensor.get();
    }

    public boolean getIfFull() {
        return !frontSensor.get() && !topSensor.get();
    }

    public void run() {
        speed = Constants.MAGAZINE_SPEED;
    }

    public void rejectCargo() {
        speed = -Constants.MAGAZINE_SPEED;
        runContinuous = true;
    }

    public void stop() {
        speed = 0;
        runContinuous = false;
    }

    public void loadContinuous() {
            runContinuous = true;
            speed = Constants.MAGAZINE_SPEED;
    }

    @Override
    public void periodic() {

        if (runContinuous) {
            magMotor.set(-speed); // Run Continuously for Shooting
        } else {

            if (!topSensor.get()) {
                speed = 0;
            } else if (!frontSensor.get()) {
                speed = Constants.MAGAZINE_SPEED;
            }

            magMotor.set(-speed);

            SmartDashboard.putBoolean("Top Sensor", !topSensor.get());
            SmartDashboard.putBoolean("Front Sensor", !frontSensor.get());
        }
    }
}