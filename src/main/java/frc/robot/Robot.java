// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

// import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  XboxController m_Xbox = new XboxController(0);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    m_robotContainer.startTeleop();
  }
  
  DoubleSolenoid m_intakeDeployment = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  boolean isDeployed = false;

  @Override
  public void teleopPeriodic() {
    if (m_Xbox.getAButton() == true) {
      m_intakeDeployment.set(DoubleSolenoid.Value.kForward);
      isDeployed = true;
    }
    else if (m_Xbox.getBButton() == true) {
      m_intakeDeployment.set(DoubleSolenoid.Value.kReverse);
      isDeployed = false;
    }
    System.out.println(isDeployed);
    }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }
}
