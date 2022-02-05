package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;

public class ClimberSubsystem extends SubsystemBase {
    private boolean debug = false;

    public static final int CLIMBER_DRIVE_MOTOR = 16;
    public static final int CLIMBER_DEPLOY = 0;
    public static final int CLIMBER_RETRACT = 1;

    private static final CANSparkMax m_climber = new CANSparkMax(CLIMBER_DRIVE_MOTOR, MotorType.kBrushless);
    private static final DoubleSolenoid m_climberDeploy 
                            = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                 CLIMBER_DEPLOY, 
                                                 CLIMBER_RETRACT);

    public ClimberSubsystem() {
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    @Override
    public void periodic() {
        Library.pushDashboard("Climber Periodic", "I am here", debug);

    }

    public void deployIntake() {
        m_climberDeploy.set(DoubleSolenoid.Value.kForward);
        Library.pushDashboard("Climber Deployment state", "deployed", debug);
    }

    public void retractIntake() {
        m_climberDeploy.set(DoubleSolenoid.Value.kReverse);
        Library.pushDashboard("Climber Deployment state", "retracted", debug);
    }

    public void neutralIntake() {
        m_climberDeploy.set(DoubleSolenoid.Value.kOff);
        Library.pushDashboard("Climber Deployment state", "neutral", debug);
    }

    public void runClimber() {

    }

    public void stopClimber() {

    }

}