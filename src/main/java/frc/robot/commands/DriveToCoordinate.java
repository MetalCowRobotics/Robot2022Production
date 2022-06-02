package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToCoordinate extends CommandBase {

    private final double kP = 0.3;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double iZ = 0.05;

    private final double POSITION_TOLERANCE = .1;
    
    private PIDController angleController;
    private PIDController xController;
    private PIDController yController;

    private double[] position;

    private DrivetrainSubsystem m_drivetrain;

    public DriveToCoordinate(DrivetrainSubsystem drive, double x, double y) {

        angleController = new PIDController(kP, kI, kD);
        angleController.setIntegratorRange(-iZ, iZ);
        angleController.setSetpoint(0);
        angleController.setTolerance(POSITION_TOLERANCE);

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

    @Override
    public void execute() {
        position = m_drivetrain.getCoordinate();

        double thetaComponent = angleController.calculate(m_drivetrain.getGyroscopeRotation().getRadians());

        double xComponent = xController.calculate(position[0]);
        double yComponent = yController.calculate(position[1]);

        SmartDashboard.putNumber("xComp", xComponent);
        SmartDashboard.putNumber("yComp", yComponent);
        
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xComponent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                yComponent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                thetaComponent * 15,
                m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint()) && angleController.atSetpoint();
    }

}