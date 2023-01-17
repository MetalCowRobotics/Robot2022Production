// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

// import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  TalonSRX m_talon15 = new TalonSRX(15);

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // m_talon15.set(ControlMode.PercentOutput, 0.5);
    m_talon15.set(ControlMode.MotionMagic, 0.5);

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    m_robotContainer.startTeleop();
  }

  @Override
  public void teleopPeriodic() {
    // CommandScheduler.getInstance().run();
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
