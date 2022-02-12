package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToCoordinate extends CommandBase {

    private final double kP = 0.4;
    private final double kI = 0.1;
    private final double kD = 0.1;
    private final double iZ = 0.05;

    private final double CORRECTION_MAX = 0.3;

    private final double POSITION_TOLERANCE = 0.05;

    private double MAX_SPEED = 0.2; 

    private double targetX;
    private double targetY; 
    
    private PIDController angleController;
    private double[] position;

    private DrivetrainSubsystem m_drivetrain;

    public DriveToCoordinate(DrivetrainSubsystem drive, double x, double y) {

        angleController = new PIDController(kP, kI, kD);
        angleController.setIntegratorRange(-iZ, iZ);
        angleController.setSetpoint(0);
        angleController.setTolerance(POSITION_TOLERANCE);

        position = drive.getCoordinate();

        m_drivetrain = drive;

        targetX = x * 0.5;
        targetY = y * 0.5;

        addRequirements(drive);
    }

    private double limit(double value, double range) {
        if (value < -range) {
            return -range;
        } else if (value > range) {
            return range;
        } else {
            return value;
        }
    }

    @Override
    public void execute() {
        position = m_drivetrain.getCoordinate();

        double thetaComponent = limit(angleController.calculate(m_drivetrain.getGyroscopeRotation().getRadians()), CORRECTION_MAX);

        double xComponentActual = targetX - position[0];
        double yComponentActual = targetY - position[1];

        double angle = Math.atan(yComponentActual / xComponentActual);

        double xComponent = copySign(MAX_SPEED * Math.cos(angle), xComponentActual);
        double yComponent = copySign(MAX_SPEED * Math.sin(angle), yComponentActual);

        // SmartDashboard.putNumber("xComp", xComponent);
        // SmartDashboard.putNumber("yComp", yComponent);
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    xComponent,
                    yComponent,
                    thetaComponent,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    private double copySign(double value, double sign) {
        double multiplier;
        try {
            multiplier = sign / Math.abs(sign);
        } catch (ArithmeticException AE) {
            multiplier = 1;
        }
        return value * multiplier;

    }

    public double getDistance() {
        double x = Math.abs(position[0] - targetX);
        double y = Math.abs(position[1] - targetY);
        // SmartDashboard.putNumber("distance from target", Math.hypot(x, y));
        return Math.hypot(x, y);
    }

    @Override
    public boolean isFinished() {
        return getDistance() < POSITION_TOLERANCE;
    }

}
