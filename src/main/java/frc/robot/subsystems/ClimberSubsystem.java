package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;

public class ClimberSubsystem extends SubsystemBase {
    private boolean debug = false;

    private static final CANSparkMax m_climber_1 = new CANSparkMax(Constants.CLIMBER_DRIVE_MOTOR_1,
            MotorType.kBrushless);
    private static final CANSparkMax m_climber_2 = new CANSparkMax(Constants.CLIMBER_DRIVE_MOTOR_2,
            MotorType.kBrushless);
    private static final DoubleSolenoid m_climberDeploy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.CLIMBER_DEPLOY, Constants.CLIMBER_RETRACT);
    private RelativeEncoder m_climbEncoder = m_climber_1.getEncoder();

    private double climbSpeed = 0;
    private boolean fieldMode = true;
    private double startPosition;

    public ClimberSubsystem() {
        m_climber_2.follow(m_climber_1);
        startPosition = m_climbEncoder.getPosition();
        SmartDashboard.putNumber("start climb", startPosition);
        SmartDashboard.putNumber("climber limit", 69);
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    @Override
    public void periodic() {
        m_climber_1.set(climbSpeed);
        SmartDashboard.putNumber("climber", m_climber_1.getEncoder().getPosition());
    }

    public void switchFieldMode() {
        fieldMode = !fieldMode;
        SmartDashboard.putBoolean("Field Mode", fieldMode);
    }

    public void retractClimber() {
        if (!fieldMode) {
            m_climberDeploy.set(DoubleSolenoid.Value.kForward);
            Library.pushDashboard("Climber Deployment state", "deployed", debug);
        }
    }

    public void deployClimber() {
        if (!fieldMode) {
            m_climberDeploy.set(DoubleSolenoid.Value.kReverse);
            Library.pushDashboard("Climber Deployment state", "retracted", debug);
        }
    }

    public void extendClimberMotor() {
        if (!fieldMode) {
            // SmartDashboard.putNumber("difference", Math.abs(m_climbEncoder.getPosition() - startPosition))
            if (Math.abs(m_climbEncoder.getPosition() - startPosition) < SmartDashboard.getNumber("climber limit", 69)) {
                climbSpeed = -Constants.CLIMB_SPEED_OUT;
            } else {
                climbSpeed = 0;
            }
        }
    }

    public void retractClimberMotor() {
        if (!fieldMode) {
            climbSpeed = Constants.CLIMB_SPEED_IN;
        }
    }

    public void stopClimberMotor() {
        climbSpeed = 0;
    }

}