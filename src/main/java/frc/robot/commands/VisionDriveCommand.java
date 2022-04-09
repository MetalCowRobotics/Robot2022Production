package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionDriveCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private VisionSubsystem m_VisionSubsystem;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;
    private DoubleSupplier m_rotationSupplier;

    private double kP = 0.8;
    private double kI = 0.0;
    private double kD = 0.15;

    private PIDController targetingPidController = new PIDController(kP, kI, kD);

    private DoubleSupplier controllerRotationSupplier;
    private DoubleSupplier visionRotationSupplier = () -> {
        if (Math.abs(m_VisionSubsystem.getCurrentYaw() - m_VisionSubsystem.getTargetYaw()) < 1) {
            return 0.0;
        } else {
            return Math.toRadians(targetingPidController.calculate(m_VisionSubsystem.getCurrentYaw())) * 15;
        }
    };

    public VisionDriveCommand(DrivetrainSubsystem drivetrainSubsystem, 
                                VisionSubsystem visionSubsystem,
                                DoubleSupplier translationXSupplier,
                                DoubleSupplier translationYSupplier,
                                DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_VisionSubsystem = visionSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.controllerRotationSupplier = rotationSupplier;
        this.m_rotationSupplier = controllerRotationSupplier;

        SmartDashboard.putNumber("Targeting kP", kP);
        SmartDashboard.putNumber("Targeting kI", kI);
        SmartDashboard.putNumber("Targeting kD", kD);

        addRequirements(drivetrainSubsystem);
        addRequirements(visionSubsystem);
    }

    @Override
    public void execute() {
        
        targetingPidController.setP(SmartDashboard.getNumber("Targeting kP", kP));
        targetingPidController.setI(SmartDashboard.getNumber("Targeting kI", kI));
        targetingPidController.setD(SmartDashboard.getNumber("Targeting kD", kD));

        targetingPidController.setSetpoint(0);

        if (m_VisionSubsystem.isTargeting()) {
            m_rotationSupplier = visionRotationSupplier;
        } else {
            m_rotationSupplier = controllerRotationSupplier;
        }
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
        SmartDashboard.putNumber("encoder", m_drivetrainSubsystem.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
