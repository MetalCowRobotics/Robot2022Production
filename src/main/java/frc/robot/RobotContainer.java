// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RunMotor;
import frc.robot.commands.RunOtherMotor;
import frc.robot.commands.StopMotor;
import frc.robot.commands.StopOtherMotor;
import frc.robot.subsystems.Cim;
import frc.robot.subsystems.Cim2;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Sensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  
final XboxController driverControls = new XboxController(0);

//SlewRateFilter controlls our ramps to the drivetrain
final SlewRateLimiter lxFilter = new SlewRateLimiter(60);
final SlewRateLimiter lyFilter = new SlewRateLimiter(60);
final SlewRateLimiter rxFilter = new SlewRateLimiter(60);
  // private final Cim m_cim = new Cim();
  // private final Cim2 m_cim2 = new Cim2();

  // Sensor m_sensor = new Sensor();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // BooleanSupplier leftHandX = () -> modifyAxis(-driverControls
    // .getY(GenericHID.Hand.kLeft)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> lxFilter.calculate(-modifyAxis(driverControls.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND)),
            () -> lyFilter.calculate(modifyAxis(driverControls.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND)),
            () -> rxFilter.calculate(-modifyAxis(driverControls.getRightX() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND))));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  SequentialCommandGroup blinkMotors = new SequentialCommandGroup();
  
  private void configureButtonBindings() {
    // blinkMotors.addCommands(new RunOtherMotor(m_cim2), new StopOtherMotor(m_cim2), new RunMotor(m_cim), new StopMotor(m_cim));
    // Back button zeros the gyroscope
    new Button(driverControls
    ::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // new Button(driverControls
    // ::getAButton)
    // .whileHeld(new RunMotor(m_cim));
    // new Button(driverControls
    // ::getAButton)
    // .whenReleased(new StopMotor(m_cim));

    // new Button(m_sensor::objectInFront).whileHeld(new ParallelCommandGroup(new RunOtherMotor(m_cim2), new StopMotor(m_cim)));
    // new Button(m_sensor::objectInFront).whenReleased(blinkMotors);
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
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
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
