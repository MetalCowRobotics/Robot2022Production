package frc.robot;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import frc.robot.RobotContainer;

public class testDeadband {
  public static final double DELTA = 1e-2;

  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @org.junit.Test // marks this method as a test
  public void deadband() {
    double adjustedValue = RobotContainer.deadband(.5, .2);
    assertEquals(.5, adjustedValue, DELTA); 
  }

  @Test
  public void atZero() {
    double adjustedValue = RobotContainer.deadband(0.0, .2);
    assertEquals(0, adjustedValue, DELTA); 
  }

}