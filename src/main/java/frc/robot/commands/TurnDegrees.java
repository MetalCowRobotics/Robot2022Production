package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnDegrees extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;
    private double turnSpeed = Math.PI * 1.9;
    private double angle;

    public TurnDegrees(DrivetrainSubsystem drive, double angle, double direction) {
        this.m_drivetrain = drive;
        this.angle = (drive.getGyroscopeRotation().getDegrees() + angle) % 360;
        SmartDashboard.putNumber("goal angle", angle);
        if (angle < 0) {
            angle += 360;
        }
        turnSpeed *= direction;
        addRequirements(drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        // double difference = Math.abs(angle - m_drivetrain.getGyroscopeRotation().getDegrees());
        // SmartDashboard.putNumber("difference", difference);
        // double actualSpeed = Math.copySign(Math.atan(Math.toRadians(difference)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, sign) * 0.3;
        // actualSpeed = Math.max(actualSpeed, (Math.PI / 2) * 3);
        SmartDashboard.putNumber("turn speed", DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        SmartDashboard.putNumber("difference", getDifference());
        double rampScalar = 1.0 / (1 + Math.exp(-(getDifference() + 5)));
        SmartDashboard.putNumber("ramped turn speed", turnSpeed * rampScalar);
        System.out.print("dif: " + getDifference());
        System.out.print(" scalar:" + rampScalar);
        System.out.println(" speed" + turnSpeed * Math.max(rampScalar, 0.4));
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    turnSpeed,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        
        return getDifference() < 7;
    }

    public double getDifference() {
        return Math.abs(angle - m_drivetrain.getGyroscopeRotation().getDegrees());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    0,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

}
