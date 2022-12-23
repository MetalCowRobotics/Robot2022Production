package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.utils.TalonFXConfigs;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
  private static final double WHEEL_RADIUS = 0.0508;
  private static final double ENCODER_RESOLUTION = 4096.0;
  private static final double DRIVE_REDUCTION = 1.0 / 6.12;
  private static final double STEER_REDUCTION = 1.0 / 12.8;

  private static final double DRIVE_DISTANCE_PER_TICK = (1 / ENCODER_RESOLUTION) * ( (WHEEL_RADIUS * Math.PI) * DRIVE_REDUCTION);
  private static final double STEER_RADIANS_PER_TICK = (1 / ENCODER_RESOLUTION) * ( (2 * Math.PI) * STEER_REDUCTION);

  private static final double kModuleMaxAngularVelocity = 2 * Math.PI;
  private static final double kModuleMaxAngularAcceleration =
    2 * Math.PI; // radians per second squared

  private TalonFX m_driveMotor;
  private TalonFX m_steeringMotor;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
    1,
    0,
    0,
    new TrapezoidProfile.Constraints(
      kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration
    )
  );

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public SwerveModule(int driveMotorCanID, int steeringMotorCanID) {
    m_driveMotor = new TalonFX(driveMotorCanID);
    m_driveMotor.configAllSettings(TalonFXConfigs.driveMotorTalonFXConfig());

    m_steeringMotor = new TalonFX(steeringMotorCanID);
    m_steeringMotor.configAllSettings(TalonFXConfigs.steerMotorTalonFXConfig());

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveSpeed(), 
      new Rotation2d(getSteerAngle())
    );
  }

  // get drive speed 
  public double getDriveSpeed() {
    return m_driveMotor.getSelectedSensorVelocity() * DRIVE_DISTANCE_PER_TICK * 0.1;
  }

  // get steering angle, not absolute
  public double getSteerAngle() {
      return m_steeringMotor.getSelectedSensorPosition() * STEER_RADIANS_PER_TICK;
  }
  // get steering speed
  public double getSteerSpeed() {
    return m_steeringMotor.getSelectedSensorVelocity() * STEER_RADIANS_PER_TICK * 0.1;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState, 
      new Rotation2d(getSteerAngle())
    );

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(
      getDriveSpeed(), 
      state.speedMetersPerSecond
    );
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(
      getSteerAngle(), 
      state.angle.getRadians()
    );
    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
    // Convert PID voltages to motor output percentages
    double drivePercent = voltageToPercent(driveOutput + driveFeedforward);
    double steerPercent = voltageToPercent(turnOutput + turnFeedforward);

    // Set motor output
    m_driveMotor.set(
      TalonFXControlMode.PercentOutput,
      drivePercent
    );
    m_steeringMotor.set(
      TalonFXControlMode.PercentOutput,
      steerPercent
    );
  }

  // Converts PID-Calculated voltage into motor output percentage
  double voltageToPercent(double voltage) {
    double percent = voltage / 12.0;
    percent = Math.max(percent, -1.0);
    percent = Math.min(percent, 1.0);
    return percent;
  }
}
