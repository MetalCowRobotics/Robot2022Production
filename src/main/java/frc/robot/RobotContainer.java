// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoDelay;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveToCoordinate;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.Magazine;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxController driverControls = new XboxController(0);
  private final XboxController operatorControls = new XboxController(1);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final DoDelay m_ShooterSubSystem = new ();
  private final Magazine m_Magazine = new Magazine();
  private final DrivetrainSubsystem m_doDelay = new DrivetrainSubsystem();
  private final ShootBall m_shootball = new ShootBall(m_Magazine, m_ShooterSubSystem, m_drivetrainSubsystem);
  SendableChooser m_chooser = new SendableChooser();

  private final double CONTROLLER_DEADBAND = 0.1;

  private boolean controllerOptions56 = false;

  private boolean RT = (driverControls.getLeftTriggerAxis() > 0.7);
  
  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(deadband(driverControls.getLeftX(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> modifyAxis(deadband(driverControls.getLeftY(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(deadband(driverControls.getRightX(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));
      // Configure the button bindings
      configureButtonBindings();

      m_chooser.setDefaultOption("Drive to -1,0", new DriveToCoordinate(m_drivetrainSubsystem, -1, 0));
      m_chooser.addOption("Drive to 1,0", new DriveToCoordinate(m_drivetrainSubsystem, 1, 0));
      m_chooser.addOption("Drive to 0,-1", new DriveToCoordinate(m_drivetrainSubsystem, 0, -1));
      m_chooser.addOption("Drive to 0,1", new DriveToCoordinate(m_drivetrainSubsystem, 0, 1));
      m_chooser.addOption("Delay Drive Forward", m_shooterCommGroup);

      SmartDashboard.putData("Autonomous Command", m_chooser);
  }

  public Command getAutoCommand(){
    return (Command) m_chooser.getSelected();
  }

  private void configureButtonBindings() {

    if (controllerOptions56)  {
      // superSprint
      // new Button(RT).whenPressed(m_drivetrainSubsystem::)
      // sprint
      new Button (driverControls::getRightBumper).whenPressed(m_drivetrainSubsystem::sprint);
      new Button(driverControls::getRightBumper).whenReleased(m_drivetrainSubsystem::resetSpeed);

      // crawl
      new Button(driverControls::getYButton).whenPressed(m_drivetrainSubsystem::crawl);
      new Button(driverControls::getYButton).whenReleased(m_drivetrainSubsystem::resetSpeed);

      // gyro
      new Button(driverControls::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

      // intake
      new Button(driverControls::get)


      
      



    

    } else {
    

	  //Reset Gyro
    new Button(driverControls::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

	  //Crawl
    new Button(driverControls::getLeftBumper).whenPressed(m_drivetrainSubsystem::crawl);
    new Button(driverControls::getLeftBumper).whenReleased(m_drivetrainSubsystem::resetSpeed);

	  //Sprint
	  new Button(driverControls::getRightBumper).whenPressed(m_drivetrainSubsystem::sprint);
    new Button(driverControls::getRightBumper).whenReleased(m_drivetrainSubsystem::resetSpeed);

    //Switch Field Mode
	  new Button(operatorControls::getBackButton).whenPressed(m_climberSubsystem::switchFieldMode);

		new Button(operatorControls::getRightBumper).whenPressed(m_climberSubsystem::extendClimberMotor);
		new Button(operatorControls::getRightBumper).whenReleased(m_climberSubsystem::stopClimberMotor);

		new Button(operatorControls::getLeftBumper).whenPressed(m_climberSubsystem::retractClimberMotor);
		new Button(operatorControls::getLeftBumper).whenReleased(m_climberSubsystem::stopClimberMotor);

		new Button(operatorControls::getAButton).whenPressed(m_climberSubsystem::deployClimber);
		new Button(operatorControls::getBButton).whenPressed(m_climberSubsystem::retractClimber);

    //Shoot
    new Button(operatorControls::getRightBumper).whenPressed(m_ShooterSubSystem::run);
    new Button(operatorControls::getRightBumper).whenReleased(m_ShooterSubSystem::stop);

    //Shooter Command Group
    new Button(operatorControls::getXButton).whenPressed(new ShootBall(m_Magazine, m_ShooterSubSystem, m_drivetrainSubsystem));

    }

    // SmartDashboard.putData("Prepare to Gather", new PrepareIntakeToGather(m_intakeSubsystem));
    SmartDashboard.putData("Retract Intake", new InstantCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem));
    SmartDashboard.putData("Neutral Intake", new InstantCommand(m_intakeSubsystem::neutralIntake, m_intakeSubsystem));
    SmartDashboard.setDefaultNumber("x Amount", 0);
    SmartDashboard.setDefaultNumber("y Amount", 0);
    // SmartDashboard.putData("DriveStraight", new DriveStraight(0, 0.3, m_drivetrainSubsystem, 4/*SmartDashboard.getNumber("Drive Amount", 0)*/));
    SmartDashboard.putData("Drive to Coord", new DriveToCoordinate(m_drivetrainSubsystem, SmartDashboard.getNumber("x Amount", 0), SmartDashboard.getNumber("y Amount", 0)));
  }

  private boolean LT() {

    double TriggerL = driverControls.getLeftTriggerAxis();

    double T = 0.7;

    return TriggerL > T;

  }
  private boolean RT() {

    double TriggerR = driverControls.getRightTriggerAxis();

    double C = 0.7;

    return TriggerR > C;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command autoCommand = new DriveStraight(0, 0.3, m_drivetrainSubsystem, 12);
    Command autoCommand = new DriveToCoordinate(m_drivetrainSubsystem, 0, 1);
    // An ExampleCommand will run in autonomous
    return autoCommand;
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
  public DrivetrainSubsystem getDrivetrain() {
    return m_drivetrainSubsystem;
  }
}
