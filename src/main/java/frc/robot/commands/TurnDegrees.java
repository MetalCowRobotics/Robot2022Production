package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnDegrees extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;
    private double turnSpeed = Math.PI / 8;
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
        SmartDashboard.putNumber("turn speed", turnSpeed / DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    turnSpeed / m_drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return getDifference() < 2;
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
