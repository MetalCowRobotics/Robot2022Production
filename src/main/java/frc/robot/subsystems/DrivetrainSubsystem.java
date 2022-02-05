// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

	final double SPRINT_SCALAR = 1.5;
	final double BASE_SPEED = 0.5;
	final double CRAWL_SCALAR = 0.2;

	public double driveSpeed = BASE_SPEED;

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
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

	// By default we use a Pigeon for our gyroscope. But if you use another
	// gyroscope, like a NavX, you can change this.
	// The important thing about how you configure your gyroscope is that rotating
	// the robot counter-clockwise should
	// cause the angle reading to increase until it wraps back over to zero.
	// FIXME Remove if you are using a Pigeon
	private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
	// FIXME Uncomment if you are using a NavX
	// private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
	// connected over MXP

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		// There are 4 methods you can call to create your swerve modules.
		// The method you use depends on what motors you are using.
		//
		// Mk3SwerveModuleHelper.createFalcon500(...)
		// Your module has two Falcon 500s on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createNeo(...)
		// Your module has two NEOs on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createFalcon500Neo(...)
		// Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
		// and the NEO is for steering.
		//
		// Mk3SwerveModuleHelper.createNeoFalcon500(...)
		// Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
		// Falcon 500 is for steering.
		//
		// Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
		// class.

		// By default we will use Falcon 500s in standard configuration. But if you use
		// a different configuration or motors
		// you MUST change it. If you do not, your code will crash on startup.
		// FIXME Setup motor configuration
		m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				// This can either be STANDARD or FAST depending on your gear configuration
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				// This is the ID of the drive motor
				FRONT_LEFT_MODULE_DRIVE_MOTOR,
				// This is the ID of the steer motor
				FRONT_LEFT_MODULE_STEER_MOTOR,
				// This is the ID of the steer encoder
				FRONT_LEFT_MODULE_STEER_ENCODER,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				FRONT_LEFT_MODULE_STEER_OFFSET);

		// We will do the same for the other modules
		m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				FRONT_RIGHT_MODULE_STEER_MOTOR,
				FRONT_RIGHT_MODULE_STEER_ENCODER,
				FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				BACK_LEFT_MODULE_DRIVE_MOTOR,
				BACK_LEFT_MODULE_STEER_MOTOR,
				BACK_LEFT_MODULE_STEER_ENCODER,
				BACK_LEFT_MODULE_STEER_OFFSET);

		m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				BACK_RIGHT_MODULE_DRIVE_MOTOR,
				BACK_RIGHT_MODULE_STEER_MOTOR,
				BACK_RIGHT_MODULE_STEER_ENCODER,
				BACK_RIGHT_MODULE_STEER_OFFSET);
	}

	public void zeroGyroscope() {
		m_pigeon.setFusedHeading(0.0);
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
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

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
		double drivetrainScalar = SmartDashboard.getNumber("Drivetrain Scalar", 0.5);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE * driveSpeed * drivetrainScalar,
				states[3].angle.getRadians());
	}
}
