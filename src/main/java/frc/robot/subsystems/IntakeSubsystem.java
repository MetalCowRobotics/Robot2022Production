package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Library;


public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;

    public static final int INTAKE_ROLLER_CAN_NUM = 1;
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    private double motorSpeed = 0;

    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM);
    private static final DoubleSolenoid m_intakeDeployment 
                            = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                 Constants.INTAKE_DEPLOYMENT_EXTEND, 
                                                 Constants.INTAKE_DEPLOYMENT_RETRACT);

    public IntakeSubsystem() {
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void run(){
        motorSpeed = 0.05;
    }

    public void stop(){
        motorSpeed = 0;
    }

    @Override
    public void periodic() {
        Library.pushDashboard("IntakePeriodic", "I am here", debug);
        intakeMotor.set(motorSpeed);
    }

    public void deployIntake() {
        m_intakeDeployment.set(DoubleSolenoid.Value.kForward);
        Library.pushDashboard("IntakeDeployment state", "deployed", debug);
    }

    public void retractIntake() {
        m_intakeDeployment.set(DoubleSolenoid.Value.kReverse);
        Library.pushDashboard("IntakeDeployment state", "retracted", debug);
    }

    public void neutralIntake() {
        m_intakeDeployment.set(DoubleSolenoid.Value.kOff);
        Library.pushDashboard("IntakeDeployment state", "neutral", debug);
    }

    public void turnIntakOn() {

    }

    public void turnIntakeOff() {

    }

}