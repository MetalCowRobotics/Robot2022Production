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
    private double speed;
    private double angle;
    private double sign;

    public TurnDegrees(DrivetrainSubsystem drive, double angle, double direction) {
        this.m_drivetrain = drive;
        this.angle = angle;
        this.sign = direction;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double actualSpeed = Math.copySign(Math.atan(Math.abs(angle - m_drivetrain.getGyroscopeRotation().getDegrees()) / 180) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, sign);
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    actualSpeed,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(angle - m_drivetrain.getGyroscopeRotation().getDegrees()) < 2;
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
