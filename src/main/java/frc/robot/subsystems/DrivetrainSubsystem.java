package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    SmartDashboard.putNumber("front right speed", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("back left speed", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("back right speed", states[3].speedMetersPerSecond);
  }

  public void drive(double x, double y, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x, 
      y, 
      rotation,
      Rotation2d.fromDegrees(m_pigeon.getYaw())
    );

    states = m_kinematics.toSwerveModuleStates(speeds);
  }
}