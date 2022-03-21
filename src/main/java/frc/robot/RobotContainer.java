// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoDelay;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveToCoordinate;
// import frc.robot.com_smmands.LoadBall;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ShootBall;
import frc.robot.commands.StartGathering;
import frc.robot.commands.StartIntakeWheels;
import frc.robot.commands.StartMagazine;
import frc.robot.commands.StartShooterWheel;
import frc.robot.commands.StartShooterWheel;
import frc.robot.commands.StopGathering;
import frc.robot.commands.StopIntakeWheels;
import frc.robot.commands.StopMagazine;
import frc.robot.commands.StopShooterWheel;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class RobotContainer {
  private final XboxController driverControls = new XboxController(0);
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final MagazineSubsystem m_magazineSubsystem = new MagazineSubsystem();
  private double delay = 0;
  private final ShootBall m_shootball = new ShootBall(m_ShooterSubsystem, m_drivetrainSubsystem, delay);
  SendableChooser m_chooser = new SendableChooser();

  private final double CONTROLLER_DEADBAND = 0.2;

  private final Command LOW_BALL_2_BALL = new SequentialCommandGroup(
    new StartGathering(m_intakeSubsystem),
    new StartGathering(m_intakeSubsystem),
    new DriveStraight(90, 0.45, m_drivetrainSubsystem, 48),
    new DoDelay(0.5),
    new StopGathering(m_intakeSubsystem),
    // new DriveStraight(0, 0.6, m_drivetrainSubsystem, 30),
    new ParallelCommandGroup(
      new StartShooterWheel(m_ShooterSubsystem),
      new TurnDegrees(m_drivetrainSubsystem, 180, -1)
    ),
    new DriveStraight(19.65 + 270, 0.6, m_drivetrainSubsystem, Math.hypot(30, 84)),
 
    new DoDelay(1), 
    new StartMagazine(m_magazineSubsystem)
  );

  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(deadband(driverControls.getLeftX(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(deadband(driverControls.getLeftY(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(deadband(driverControls.getRightX(), CONTROLLER_DEADBAND) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.75)));
      // Configure the button bindings
      configureButtonBindings();

      // m_chooser.setDefaultOption("Drive to -1,0", new DriveToCoordinate(m_drivetrainSubsystem, -1, 0));
      // m_chooser.addOption("Drive to 1,0", new DriveToCoordinate(m_drivetrainSubsystem, 1, 0));
      // m_chooser.addOption("Drive to 0,-1", new DriveToCoordinate(m_drivetrainSubsystem, 0, -1));
      // m_chooser.addOption("Drive to 0,1", new DriveToCoordinate(m_drivetrainSubsystem, 0, 1));
      // m_chooser.addOption("Delay Drive Forward", m_shootball);
      SmartDashboard.putNumber("Delay", delay);

      SmartDashboard.putData("Autonomous Command", m_chooser);
  }

  public Command getAutoCommand(){
    // m_drivetrainSubsystem.zeroGyroscope();
    // return new DriveStraight(85, 0.3, m_drivetrainSubsystem, 30);
    // return new TurnDegrees(m_drivetrainSubsystem, 180, 1);
    // return new SequentialCommandGroup(new StartShooterWheel(m_ShooterSubsystem), new DoDelay(3), new StartMagazine(m_magazineSubsystem), new DoDelay(2), new DriveStraight(90, 0.3, m_drivetrainSubsystem, 90));
    return LOW_BALL_2_BALL;
    // return new SequentialCommandGroup(
    //   new DoDelay(0.5),
    //   new StartGathering(m_intakeSubsystem),
    //   new StartGathering(m_intakeSubsystem),
    //   new DriveStraight(90, 0.3, m_drivetrainSubsystem, 52),
    //   new StopGathering(m_intakeSubsystem),
    //   new TurnDegrees(m_drivetrainSubsystem, 175, -1),
    //   new DriveStraight(270, 0.3, m_drivetrainSubsystem, 86),

    //   new StartShooterWheel(m_ShooterSubsystem), 
    //   new DoDelay(3), 
    //   new StartMagazine(m_magazineSubsystem)
    //   // new DoDelay(0.75),
    //   // new StopMagazine(m_magazineSubsystem),
    //   // new DoDelay(1),
    //   // new StartMagazine(m_magazineSubsystem)
    // );
  }

  private void configureButtonBindings() {

    new Button(m_magazineSubsystem::getIfFull).whenPressed(new StopGathering(m_intakeSubsystem));

    //Driver
		//Reset Gyro
		Constants.CONT_RESET_GYRO.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

		//Crawl
		Constants.CONT_CRAWL.whenPressed(m_drivetrainSubsystem::crawl);
		Constants.CONT_CRAWL.whenReleased(m_drivetrainSubsystem::resetSpeed);

		//Sprint
		Constants.CONT_SPRINT.whenPressed(m_drivetrainSubsystem::sprint);
		Constants.CONT_SPRINT.whenReleased(m_drivetrainSubsystem::resetSpeed);

    Constants.CONT_INTAKE_DEPLOY.whenPressed(new StartGathering(m_intakeSubsystem));
		Constants.CONT_INTAKE_RETRACT.whenPressed(new StopGathering(m_intakeSubsystem));

	//Operator
		//Switch Field Mode
		Constants.CONT_SWITCH_FIELD_MODE.whenPressed(m_climberSubsystem::switchFieldMode);
    Constants.CONT_SWITCH_FIELD_MODE.whenPressed(m_ShooterSubsystem::switchFieldMode);

		//Climb
		Constants.CONT_CLIMBER_UP.whileHeld(m_climberSubsystem::extendClimberMotor);
		Constants.CONT_CLIMBER_UP.whenReleased(m_climberSubsystem::stopClimberMotor);

		Constants.CONT_CLIMBER_DOWN.whenPressed(m_climberSubsystem::retractClimberMotor);
		Constants.CONT_CLIMBER_DOWN.whenReleased(m_climberSubsystem::stopClimberMotor);

		Constants.CONT_CLIMBER_OUT.whenPressed(m_climberSubsystem::deployClimber);
		Constants.CONT_CLIMBER_IN.whenPressed(m_climberSubsystem::retractClimber);

		//Shoot
    Constants.CONT_SHOOTER_LOW.whenPressed(m_ShooterSubsystem::shootLow);
		Constants.CONT_SHOOTER_FIRE.whenPressed(() -> {
      m_magazineSubsystem.loadContinuous();
      m_intakeSubsystem.run();
    });


		Constants.CONT_SHOOTER_LOW.whenReleased(m_ShooterSubsystem::stop);
		Constants.CONT_SHOOTER_FIRE.whenReleased(() -> {
      m_magazineSubsystem.stop();
      m_intakeSubsystem.stop();
    });


    //Shooter Command Group
    delay = SmartDashboard.getNumber("Delay", delay);
    // new Button(operatorControls::getXButton).whenPressed(new ShootBall(m_ShooterSubsystem, m_drivetrainSubsystem, delay));

    

    // SmartDashboard.putData("Prepare to Gather", new PrepareIntakeToGather(m_intakeSubsystem));
    // SmartDashboard.putData("Retract Intake", new InstantCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem));
    // SmartDashboard.putData("Neutral Intake", new InstantCommand(m_intakeSubsystem::neutralIntake, m_intakeSubsystem));
    // SmartDashboard.setDefaultNumber("x Amount", 0);
    // SmartDashboard.setDefaultNumber("y Amount", 0);
    // SmartDashboard.putData("DriveStraight", new DriveStraight(0, 0.3, m_drivetrainSubsystem, 4/*SmartDashboard.getNumber("Drive Amount", 0)*/));
    // SmartDashboard.putData("Drive to Coord", new DriveToCoordinate(m_drivetrainSubsystem, SmartDashboard.getNumber("x Amount", 0), SmartDashboard.getNumber("y Amount", 0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = new DriveStraight(0, 0.3, m_drivetrainSubsystem, 12);
    // Command autoCommand = new DriveToCoordinate(m_drivetrainSubsystem, 0, 1);
    // An ExampleCommand will run in autonomous
    return null;//autoCommand;
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
    return null; //m_drivetrainSubsystem;
  }
}

