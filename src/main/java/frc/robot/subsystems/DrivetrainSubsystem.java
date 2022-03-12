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
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
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

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
//  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  final double SPRINT_SCALAR = 1.45;
  final double BASE_SPEED = 0.65;
  final double CRAWL_SCALAR = 0.2;

  public double driveSpeed = BASE_SPEED;

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
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration

        
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L3,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
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

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    m_pigeon.setYaw(90.0);

    // FIXME Uncomment if you are using a NavX
//    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
        // m_pigeon.getFusedHeading();

        SmartDashboard.putNumber("gyro angle", m_pigeon.getYaw());
        return Rotation2d.fromDegrees(Math.abs(m_pigeon.getYaw() % 360));
        // return Rotation2d.fromDegrees(m_pigeon.getAbsoluteCompassHeading());


    // FIXME Uncomment if you are using a NavX
//    if (m_navx.isMagnetometerCalibrated()) {
//      // We will only get valid fused headings if the magnetometer is calibrated
//      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
//    }
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
//    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void crawl() {
        driveSpeed = BASE_SPEED * CRAWL_SCALAR;
}

public void sprint() {
        driveSpeed = BASE_SPEED * SPRINT_SCALAR;
}

public void resetSpeed() {
        driveSpeed = BASE_SPEED;
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
        // states[0].
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
