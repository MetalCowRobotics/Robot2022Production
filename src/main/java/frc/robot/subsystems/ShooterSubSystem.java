package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    private CANSparkMax leftMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);

    private static final DoubleSolenoid hoodPosition = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
    Constants.HOOD_CLOSE_SHOT, Constants.HOOD_FAR_SHOT);

    private RelativeEncoder encoder;
    private double targetSpeed = 0;
    private SparkMaxPIDController pid;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double goalSpeed;

    private boolean fieldMode = true;

    public ShooterSubsystem() {
        pid = rightMotor.getPIDController();
        encoder = rightMotor.getEncoder();

        SmartDashboard.putNumber("Speed Correction", 0);

        // PID coefficients
        kP = 0.00012;
        kI = 0.0000005;
        kD = 0.0001;
        kIz = 3000;
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

        leftMotor.follow(rightMotor, true);
        if (hoodPosition.get().equals(DoubleSolenoid.Value.kForward)) {
            goalSpeed = Constants.SHOOTER_BASE_SPEED + 625;
        } else if (hoodPosition.get().equals(DoubleSolenoid.Value.kReverse)) {
            goalSpeed = Constants.SHOOTER_BASE_SPEED + 75;   
        } else {
            hoodFarShot();
        }
    }

    public void hoodFarShot() {
        goalSpeed = Constants.SHOOTER_BASE_SPEED + 525;
        hoodPosition.set(DoubleSolenoid.Value.kForward);
        SmartDashboard.putBoolean("Hood Up", true);
        SmartDashboard.putBoolean("Hood Down", false);
    }

    public void hoodCloseShot() {
        goalSpeed = Constants.SHOOTER_BASE_SPEED + 75;
        hoodPosition.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putBoolean("Hood Up", false);
        SmartDashboard.putBoolean("Hood Down", true);
    }

    public void shootLow() {
        if (fieldMode) {
            targetSpeed = goalSpeed + SmartDashboard.getNumber("Speed Correction", 0);
        }
    }

    public void scaleShooterSpeed(double scalar) {
        if (scalar < 1.4) {
            targetSpeed = (goalSpeed + SmartDashboard.getNumber("Speed Correction", 0)) * scalar;            
        }
    }

    public void stop() {
        targetSpeed = 0;
    }

    public void startRumble(XboxController controller) {
            controller.setRumble(RumbleType.kLeftRumble, 0.5);
            controller.setRumble(RumbleType.kRightRumble, 0.5);
    }

    public void stopRumble(XboxController controller) {
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
    }

    @Override

    public void periodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0.00012);
        double i = SmartDashboard.getNumber("I Gain", 0.0000005);
        double d = SmartDashboard.getNumber("D Gain", 0.0001);
        double iz = SmartDashboard.getNumber("I Zone", 1000);
        double ff = SmartDashboard.getNumber("Feed Forward", 0.000015);
        double max = SmartDashboard.getNumber("Max Output", -1);
        double min = SmartDashboard.getNumber("Min Output", 1);

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
        if (targetSpeed != 0) {
            pid.setReference(-targetSpeed, CANSparkMax.ControlType.kVelocity);
        } else {
            rightMotor.set(0);
        }
        SmartDashboard.putNumber("Current Speed", Math.abs(encoder.getVelocity()));
        SmartDashboard.putNumber("Target Speed", targetSpeed);
        SmartDashboard.putBoolean("isReady", isReady());
    }

    public boolean isReady() {
        if (Math.abs(encoder.getVelocity()) >= (targetSpeed - 50) && targetSpeed != 0) {
            return true;
        } else {
            return false;
        }
    }

    public void switchFieldMode() {
        fieldMode = !fieldMode;
    }

}
