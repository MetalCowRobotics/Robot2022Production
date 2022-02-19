package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubSystem extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);

    private RelativeEncoder encoder = leftMotor.getEncoder();
    private double targetSpeed = 0;
    private SparkMaxPIDController pid;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public ShooterSubSystem() {
        pid = leftMotor.getPIDController();

        SmartDashboard.putNumber("Shooter Target", 500);

        // PID coefficients
        kP = 0.00006;
        kI = 0.000001;
        kD = 0.0001;
        kIz = 10000;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        rightMotor.follow(leftMotor, true);
    }

    public void run() {
        targetSpeed = SmartDashboard.getNumber("Shooter Target", 0);
    }

    public void stop() {
        targetSpeed = 0;
    }

    @Override

    public void periodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pid.setP(p); kP = p; }
        if((i != kI)) { pid.setI(i); kI = i; }
        if((d != kD)) { pid.setD(d); kD = d; }
        if((iz != kIz)) { pid.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pid.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            pid.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        pid.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Current Speed", encoder.getVelocity());
        SmartDashboard.putBoolean("isReady", isReady());
    }

    public boolean isReady() {
        if (encoder.getVelocity() >= targetSpeed) {
            return true;
        } else {
            return false;
        }
    }

}
