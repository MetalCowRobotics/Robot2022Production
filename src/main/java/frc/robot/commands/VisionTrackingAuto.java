package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTrackingAuto extends CommandBase {
    private Timer m_timer = new Timer();
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private VisionSubsystem m_VisionSubsystem;

    private double kP = 0.3;
    private double kI = 0.15;
    private double kD = 0.0;

    private final double HAPPY_ZONE = 1;

    private PIDController targetingPidController = new PIDController(kP, kI, kD);

    private DoubleSupplier visionRotationSupplier = () -> {
        if (Math.abs(m_VisionSubsystem.getCurrentYaw() - m_VisionSubsystem.getTargetYaw()) < HAPPY_ZONE) {
            return 0.0;
        } else {
            return Math.toRadians(targetingPidController.calculate(m_VisionSubsystem.getCurrentYaw())) * 15;
        }
    };

    private double timeLimit;

    public VisionTrackingAuto(DrivetrainSubsystem drivetrainSubsystem, 
                                VisionSubsystem visionSubsystem,
                                double timeLimit) {

        this.timeLimit = timeLimit;
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_VisionSubsystem = visionSubsystem;

        SmartDashboard.putNumber("Targeting kP", kP);
        SmartDashboard.putNumber("Targeting kI", kI);
        SmartDashboard.putNumber("Targeting kD", kD);

        addRequirements(drivetrainSubsystem);
        addRequirements(visionSubsystem);
    }

    public void initialize() {
        m_timer.start();
        m_VisionSubsystem.startTargeting();
    }

    @Override
    public void execute() {
        
        targetingPidController.setP(SmartDashboard.getNumber("Targeting kP", kP));
        targetingPidController.setI(SmartDashboard.getNumber("Targeting kI", kI));
        targetingPidController.setD(SmartDashboard.getNumber("Targeting kD", kD));

        targetingPidController.setSetpoint(-m_VisionSubsystem.getTargetYaw());

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        visionRotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > timeLimit;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}