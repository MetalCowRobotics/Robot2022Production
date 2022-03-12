package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MagazineSubsystem extends SubsystemBase {
    private CANSparkMax magMotor = new CANSparkMax(Constants.MAGAZINE_MOTOR, MotorType.kBrushless);
    private DigitalInput ballSensor = new DigitalInput(Constants.MAGAZINE_SENSOR);
    private double speed = 0;
    private boolean ballOnSensor = false;
    private boolean ballPassed = false;
    private boolean lastState = false;
    private boolean runContinuous = false;

    public void run() {
        speed = Constants.MAGAZINE_SPEED;
    }

    public void stop() {
        speed = 0;
        runContinuous = false;
    }

    public boolean getState() {
        return ballSensor.get();
    }

    public void loadContinuous() {
        runContinuous = true;
        ballPassed = false;
        speed = SmartDashboard.getNumber("mag speed", 0.3);
    }

    @Override
    public void periodic() {

        if (runContinuous) {
            magMotor.set(-speed); // Run Continuously for Shooting
        } else {
            if (!ballOnSensor && ballSensor.get() && !lastState) { // Get if the ball is currently over the sensor
                ballOnSensor = true; // Change state and keep running
            } else if (ballOnSensor && ballSensor.get() != lastState) { // Get right after ball is past the sensor
                speed = 0; // Change state and stop magazine.  We are holding a ball up in the magazine
                ballOnSensor = false;
                ballPassed = true;
            } else if (!ballPassed) { // Run motor
                speed = Constants.MAGAZINE_SPEED;
                ballPassed = false;
            }

            magMotor.set(-speed);
            lastState = ballSensor.get();
        }
    }
}