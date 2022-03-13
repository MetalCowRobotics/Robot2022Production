// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public double driveSpeed = Constants.BASE_SPEED;

  private TalonFX backRightDrive;
  
  private TalonFX backLeftDrive;
  
  private TalonFX frontRightDrive;
  
  private TalonFX frontLeftDrive;

  private CANCoder frontRightEncoder;
  private CANCoder frontLeftEncoder;
  private CANCoder backRightEncoder;
  private CANCoder backLeftEncoder;
  
  private SwerveDriveOdometry odometer;
  private Pose2d position = new Pose2d();

  public DrivetrainSubsystem() {
        Translation2d backRightPosition = new Translation2d(0.292, -0.292);
        Translation2d backLeftPosition = new Translation2d(-0.292, -0.292);
        Translation2d frontRightPosition = new Translation2d(0.292, 0.292);
        Translation2d frontLeftPosition = new Translation2d(-0.292, 0.292);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition
        );

        odometer = new SwerveDriveOdometry(m_kinematics,

        Rotation2d.fromDegrees(m_pigeon.getAbsoluteCompassHeading()), new Pose2d(0.0, 0.0, new Rotation2d()));

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        backRightDrive = new TalonFX(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR);

        backLeftDrive = new TalonFX(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR);

        frontRightDrive = new TalonFX(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);

        frontLeftDrive = new TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);

        frontRightEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
        frontLeftEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
        backRightEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
        backLeftEncoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);
  
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4SwerveModuleHelper.GearRatio.L3,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );


    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L3,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET
        );
        }

  public void zeroGyroscope() {
    m_pigeon.setYaw(90);
  }

  public Rotation2d getGyroscopeRotation() {
        SmartDashboard.putNumber("gyro angle", m_pigeon.getYaw());
        return Rotation2d.fromDegrees(Math.abs(m_pigeon.getYaw() % 360));
  }

  public void crawl() {
        driveSpeed = Constants.BASE_SPEED * Constants.CRAWL_SCALAR;
}

public void sprint() {
        driveSpeed = Constants.BASE_SPEED * Constants.SPRINT_SCALAR;
}

public void resetSpeed() {
        driveSpeed = Constants.BASE_SPEED;
}

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public double getPosition() {
          return backRightDrive.getSelectedSensorPosition();
  }

  public double[] getCoordinate() {
        double[] coordinate = {position.getX(), position.getY()};
        return coordinate;
  }

  @Override
  public void periodic() {
        // SmartDashboard.putNumber("Input", backRightSteer.getSupplyCurrent());

        
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SmartDashboard.putNumber("x coordinate", getCoordinate()[0]);
    SmartDashboard.putNumber("y coordinate", getCoordinate()[1]);
//     SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

//     position = odometer.update(Rotation2d.fromDegrees(m_pigeon.getAbsoluteCompassHeading()), states[0], states[1], states[2], states[3]);

        double drivetrainScalar = SmartDashboard.getNumber("Drivetrain Scalar", 1);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
                states[0].angle.getRadians());
        
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
                states[1].angle.getRadians());
        
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
                states[2].angle.getRadians());

        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
                states[3].angle.getRadians());
//     m_backRightModule.getDriveVelocity()
        updatePosition();
  }

  public void updatePosition() {
        //   SmartDashboard.putNumber("motor speed", convertMotorVelocity(frontLeftDrive.getMotorOutputPercent() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND));
          SmartDashboard.putNumber("motor speed", frontLeftDrive.getMotorOutputPercent() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        SwerveModuleState frontLeft = new SwerveModuleState(convertMotorVelocity(frontLeftDrive.getSelectedSensorVelocity()), Rotation2d.fromDegrees((frontLeftEncoder.getPosition() + Constants.FRONT_LEFT_MODULE_STEER_OFFSET) % 360));
        SwerveModuleState frontRight = new SwerveModuleState(convertMotorVelocity(frontRightDrive.getSelectedSensorVelocity()), Rotation2d.fromDegrees((frontRightEncoder.getPosition() + Constants.FRONT_RIGHT_MODULE_STEER_OFFSET) % 360));
        SwerveModuleState backLeft = new SwerveModuleState(convertMotorVelocity(backLeftDrive.getSelectedSensorVelocity()), Rotation2d.fromDegrees((backLeftEncoder.getPosition() + Constants.BACK_LEFT_MODULE_STEER_OFFSET) % 360));
        SwerveModuleState backRight = new SwerveModuleState(convertMotorVelocity(backRightDrive.getSelectedSensorVelocity()), Rotation2d.fromDegrees((backRightEncoder.getPosition() + Constants.BACK_RIGHT_MODULE_STEER_OFFSET) % 360));
        position = odometer.update(Rotation2d.fromDegrees(m_pigeon.getAbsoluteCompassHeading()), frontLeft, frontRight, backLeft, backRight);
  }

  private double convertMotorVelocity(double ticSpeed) {
        return (ticSpeed / 100.0) * (1000.0 / 1) * (1 / 2048.0) * (1 / 6.12) * (0.319) * 2;
  }

}
