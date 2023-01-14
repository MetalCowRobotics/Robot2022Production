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
import com.ctre.phoenix.sensors.CANCoder;

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
  private CANCoder m_steerEncoder;

  private double offset;

  private final PIDController m_drivePIDController = new PIDController(0, 0, 0);

  private final PIDController m_steerPIDController = new PIDController(0.08, 0, 0.0);

  public SwerveModule(int driveMotorCanID, int steeringMotorCanID, int encoderCanID, double offset) {
    m_driveMotor = new TalonFX(driveMotorCanID);
    m_driveMotor.configAllSettings(TalonFXConfigs.driveMotorTalonFXConfig());
    m_steerEncoder = new CANCoder(encoderCanID);
    // m_steerEncoder.configMagnetOffset(-offset);

    this.offset = offset;

    m_steeringMotor = new TalonFX(steeringMotorCanID);
    m_steeringMotor.configAllSettings(TalonFXConfigs.steerMotorTalonFXConfig());

    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_steerPIDController.setTolerance(5, 100);
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

  // get steering angle, absolute
  public double getSteerAngle() {
      return Rotation2d.fromDegrees(m_steerEncoder.getAbsolutePosition()).getDegrees();
  }
  // get steering speed
  public double getSteerSpeed() {
    return m_steerEncoder.getVelocity();
  }

  public SwerveModuleState getModuleState() {
    SwerveModuleState state = new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getSteerAngle()));
    return state;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = desiredState;
    m_drivePIDController.setSetpoint(state.speedMetersPerSecond);
    m_steerPIDController.setSetpoint(state.angle.getDegrees());
    // SwerveModuleState state = SwerveModuleState.optimize(
    //   desiredState, 
    //   new Rotation2d(getSteerAngle())
    // );

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(
      getDriveSpeed(), 
      state.speedMetersPerSecond
    );

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_steerPIDController.calculate(getSteerAngle());
    turnOutput = Math.min(turnOutput, 0.1);
    turnOutput = Math.max(turnOutput, -0.1);
    
    System.out.println(turnOutput);

    // if (m_steerPIDController.atSetpoint()) {
    //   turnOutput = 0;
    // }

    // Convert PID voltages to motor output percentages
    // double drivePercent = voltageToPercent(driveOutput);
    // double steerPercent = voltageToPercent(turnOutput);

    // Set motor output
    m_driveMotor.set(
      TalonFXControlMode.PercentOutput,
      driveOutput
    );
    m_steeringMotor.set(
      TalonFXControlMode.PercentOutput,
      turnOutput
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
