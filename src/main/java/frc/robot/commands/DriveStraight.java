package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraight extends CommandBase {

    // array to store the drive vector in form <i, j>
    public double[] driveVector = new double[2];
    private DrivetrainSubsystem m_drivetrain;
    private TalonFX backRightDrive;
    private double endTicsDifference;
    private boolean firstTime = true;
    private double angle, speed, inches, startTics;

    public DriveStraight(double angle, double speed, DrivetrainSubsystem drive, double inches) {
        this.m_drivetrain = drive;
        SmartDashboard.putNumber("start tics", m_drivetrain.getPosition());
        this.inches = inches;
        this.angle = angle;
        this.speed = speed;
        // SmartDashboard.putNumber("end position", endTics);

        addRequirements(m_drivetrain);
    }

    ChassisSpeeds speeds = new ChassisSpeeds();

    @Override
    public void execute() {
        if (firstTime) {
            SmartDashboard.putNumber("first time tics", m_drivetrain.getPosition());
            startTics = m_drivetrain.getPosition();
            endTicsDifference = (inches / (4 * Math.PI)) * 12591;
            // endTics = m_drivetrain.getPosition() + (1024 / 6.86) * 6.86 * 4.28;
            

            driveVector[0] = speed * Math.cos(Math.toRadians(angle));
            driveVector[1] = speed * Math.sin(Math.toRadians(angle));
            firstTime = false;

            SmartDashboard.putNumber("end tics", endTicsDifference);
        }

        SmartDashboard.putNumber("current tics", m_drivetrain.getPosition());

        m_drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVector[0],
                        driveVector[1],
                        0,
                        m_drivetrain.getGyroscopeRotation()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_drivetrain.getPosition() - startTics) > endTicsDifference);
    }

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
