

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;


public class TestCommand extends CommandBase {
  // The subsystem the command runs on
  private final TestSubsystem m_TestSubsystem;

  public TestCommand(TestSubsystem subsystem) {
    m_TestSubsystem = subsystem;
    addRequirements(m_TestSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}