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

public class DrivePath extends CommandBase {

    private Timer m_timer = new Timer();
    private double time;
    private File csvFile;
    private Scanner lineReader;
    private DrivetrainSubsystem m_drivetrain;
    private double speed;
    private double angle;
    private double rotationSpeed;
    

    public DrivePath(DrivetrainSubsystem drive, String fileName) {
        m_drivetrain = drive;
        csvFile = new File(Filesystem.getDeployDirectory() + "/pathplanner/generatedCSV/" + fileName);
        try {
            SmartDashboard.putString("path", Filesystem.getDeployDirectory() + "/pathplanner/generatedCSV/NewPath.csv");
            lineReader = new Scanner(csvFile);
            lineReader.nextLine();
            lineReader.nextLine();
            SmartDashboard.putBoolean("found file", true);
            getNextLine();
        } catch (Exception e) {
            SmartDashboard.putBoolean("found file", false);
        }
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (m_timer.get() > time) {
            getNextLine();            
        }

        double xComponent = speed * Math.cos(angle);
        double yComponent = speed * Math.sin(angle);
        
        m_drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    xComponent,
                    yComponent,
                    rotationSpeed,
                    m_drivetrain.getGyroscopeRotation()
            )
        );
    }

    public void getNextLine() {
        String line = lineReader.nextLine();
        SmartDashboard.putString("values", line);
        String[] values = line.split(",");
        speed = Double.parseDouble(values[2]);
        time = Double.parseDouble(values[0].substring(1));
        angle = Math.toRadians(Double.parseDouble(values[4]));
        rotationSpeed = Math.toRadians(Double.parseDouble(values[values.length - 1]));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return !lineReader.hasNext();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
