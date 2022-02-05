package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;

public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = true;

    public static final int INTAKE_ROLLER_CAN_NUM = 1;
    public static final int INTAKE_DEPLOYMENT_EXTEND = 0;
    public static final int INTAKE_DEPLOYMENT_RETRACT = 1;

    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM);
    private static final DoubleSolenoid m_intakeDeployment 
                            = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                 INTAKE_DEPLOYMENT_EXTEND, 
                                                 INTAKE_DEPLOYMENT_RETRACT);

    public IntakeSubsystem() {
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    @Override
    public void periodic() {
        Library.pushDashboard("IntakePeriodic", "I am here", debug);

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