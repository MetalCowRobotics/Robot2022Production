package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnToTarget extends CommandBase {
    private DrivetrainSubsystem drive;
    private VisionSubsystem vision;

    private boolean checkState = true;

    private double kP = 0.8;
    private double kI = 0.0;
    private double kD = 0.15;

    private final double TARGETING_THRESHOLD = 1.0;

    private PIDController targetingPidController;
    
    public TurnToTarget(DrivetrainSubsystem drive, VisionSubsystem vision) {
       this.drive = drive;
       this.vision = vision;
       
       targetingPidController = new PIDController(kP, kI, kD);

       addRequirements(drive);
       addRequirements(vision);
    }

    @Override 
    public void execute() {


        double correction = targetingPidController.calculate(vision.getCurrentYaw());

        SmartDashboard.putNumber("target facing corection", Math.toRadians(correction) * 8);

        SmartDashboard.putNumber("target facing target", vision.getTargetYaw());
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 
            0, 
            Math.toRadians(correction) * 15, 
            drive.getGyroscopeRotation()
        ));
        // if (vision.isTargeting()) {
        // } else {
        //     drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        //         0, 
        //         0, 
        //         0, 
        //         drive.getGyroscopeRotation()
        //     ));
        // }
    }

    public boolean isFinished() {
        return Math.abs(vision.getCurrentYaw() - vision.getTargetYaw()) < TARGETING_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 
            0, 
            0, 
            drive.getGyroscopeRotation()
        ));
    }
}
