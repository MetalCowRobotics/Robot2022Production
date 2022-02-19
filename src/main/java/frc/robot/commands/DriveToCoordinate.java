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

    private final double POSITION_TOLERANCE = 5;

    private double MAX_SPEED = 0.3; 

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

        targetX = x * 100;
        targetY = y * 100;

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

        double angle;

        if (xComponentActual != 0) {
            angle = Math.atan(yComponentActual / xComponentActual);
        } else {
            if (yComponentActual > 0) {
                angle = 90;
            } else {
                angle = 270;
            }
        }

        double xComponent = Math.copySign(MAX_SPEED * Math.cos(angle), xComponentActual);
        double yComponent = Math.copySign(MAX_SPEED * Math.sin(angle), yComponentActual);

        SmartDashboard.putNumber("xComp", xComponent);
        SmartDashboard.putNumber("yComp", yComponent);
        
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xComponent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                yComponent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
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

    private static double modifyAxis(double value) {
        // Deadband
       // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          return value;
        } else {
          return 0.0;
        }
      }

    @Override
    public boolean isFinished() {
        return getDistance() < POSITION_TOLERANCE;
    }

}