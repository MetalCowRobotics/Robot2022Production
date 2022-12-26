package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeftModule = new SwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR);
  private final SwerveModule backLeftModule = new SwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR);
  private final SwerveModule backRightModule = new SwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR);

  private final PigeonIMU m_pigeon = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, 
    m_frontRightLocation, 
    m_backLeftLocation, 
    m_backRightLocation
  );
  SwerveModuleState[] states;

  Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d(0));

  SwerveDrivePoseEstimator m_estimator = new SwerveDrivePoseEstimator(new Rotation2d(0), initialPose, m_kinematics, VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(0.001)), VecBuilder.fill(0.001), VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(0.001)));

  Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d(0));

  public DrivetrainSubsystem() {
    m_pigeon.setYaw(0);
    drive(0, 0, 0);
  }

  @Override
  public void periodic() {
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);

    SmartDashboard.putNumber("front left speed", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("front left angle", states[0].angle.getDegrees());
    
    SmartDashboard.putNumber("front right speed", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("front right angle", states[1].angle.getDegrees());
    
    SmartDashboard.putNumber("back left speed", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("back left angle", states[2].angle.getDegrees());

    SmartDashboard.putNumber("back right speed", states[3].speedMetersPerSecond);
    SmartDashboard.putNumber("back right angle", states[3].angle.getDegrees());

    currentPose = m_estimator.update(Rotation2d.fromDegrees(m_pigeon.getYaw()), frontLeftModule.getModuleState(), frontRightModule.getModuleState(), backLeftModule.getModuleState(), backRightModule.getModuleState());
    SmartDashboard.putNumber("robot x", currentPose.getX());
    SmartDashboard.putNumber("robot y", currentPose.getY());
  }

  public void drive(double x, double y, double rotation) {
    if (Math.abs(x) < 0.1) {
      x = 0;
    }

    if (Math.abs(y) < 0.1) {
      y = 0;
    }

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x, 
      y, 
      rotation,
      Rotation2d.fromDegrees(m_pigeon.getYaw())
    );

    states = m_kinematics.toSwerveModuleStates(speeds);
  }
}