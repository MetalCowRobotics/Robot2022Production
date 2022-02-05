// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PrepareIntakeToGather;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxController driverControls = new XboxController(0);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final SparkMax m_Spark = new SparkMax(16);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(driverControls.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> modifyAxis(driverControls.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(driverControls.getRightX() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
//   SequentialCommandGroup blinkMotors = new SequentialCommandGroup();
  
  private void configureButtonBindings() {
	//Reset Gyro
    new Button(driverControls::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

	//Crawl
    new Button(driverControls::getLeftBumper).whenPressed(m_drivetrainSubsystem::crawl);
    new Button(driverControls::getLeftBumper).whenReleased(m_drivetrainSubsystem::resetSpeed);

	//Sprint
	new Button(driverControls::getRightBumper).whenPressed(m_drivetrainSubsystem::sprint);
    new Button(driverControls::getRightBumper).whenReleased(m_drivetrainSubsystem::resetSpeed);

	//Neo550 Test
    new Button(driverControls::getAButton).whenPressed(m_Spark::run);
    new Button(driverControls::getAButton).whenReleased(m_Spark::stop);

    SmartDashboard.putData("Prepare to Gather", new PrepareIntakeToGather(m_intakeSubsystem));
    SmartDashboard.putData("Retract Intake", new InstantCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem));
    SmartDashboard.putData("Neutral Intake", new InstantCommand(m_intakeSubsystem::neutralIntake, m_intakeSubsystem));
    // SmartDashboard.putData("Run SPARK", new InstantCommand(m_sparkSystem::run, m_sparkSystem));
    // SmartDashboard.putData("Stop SPARK", new InstantCommand(m_sparkSystem::stop, m_sparkSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return value;
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

   // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
}
