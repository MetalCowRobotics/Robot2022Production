package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToCoordinate extends CommandBase {

    private final double kP = 0.4;
    private final double kI = 0;
    private final double kD = 1;
    private final double iZ = 0.1;

    private final double CORRECTION_MAX = 0.2;

    private final double POSITION_TOLERANCE = 0.05;
    
    private PIDController xController;
    private PIDController yController;
    private double[] position;

    private DrivetrainSubsystem m_drivetrain;

    public DriveToCoordinate(DrivetrainSubsystem drive, double x, double y) {
        xController = new PIDController(kP, kI, kD);
        xController.setIntegratorRange(-iZ, iZ);
        xController.setSetpoint(x);
        xController.setTolerance(POSITION_TOLERANCE);
        

        yController = new PIDController(kP, kI, kD);
        yController.setIntegratorRange(-iZ, iZ);
        yController.setSetpoint(y);
        yController.setTolerance(POSITION_TOLERANCE);

        position = drive.getCoordinate();

        m_drivetrain = drive;

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
        double xComponent = limit(xController.calculate(position[0]), CORRECTION_MAX);
        double yComponent = limit(xController.calculate(position[1]), CORRECTION_MAX);
        SmartDashboard.putNumber("xComp", xComponent);
        SmartDashboard.putNumber("yComp", yComponent);
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    xComponent,
                    yComponent,
                    0,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("at coord", (xController.atSetpoint() && yController.atSetpoint()));
        return (xController.atSetpoint() && yController.atSetpoint());
    }

}
