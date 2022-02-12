package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PidTest extends CommandBase {
    private CANSparkMax testMotor;
    private PIDController rpmController;
    private static double P = .0002;
    private static double I = .00001;
    private static double D = .0001;
    private static double Iz = 0;
    public PidTest() {
        testMotor = new CANSparkMax(10, MotorType.kBrushless);
        rpmController = new PIDController(P, I, D);
        rpmController.setIntegratorRange(-Iz, Iz);
        rpmController.setSetpoint(120);
    }
    @Override
    public void execute() {
        double currentSpeed = testMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("motor rpm", currentSpeed);
        double correction = rpmController.calculate(currentSpeed);
        SmartDashboard.putNumber("neo correction", correction);
        if (correction > 0.3) {
            correction = 0.3;
        } else if (correction < -0.3) {
            correction = -0.3;
        }
        testMotor.set(correction);
    }
}
