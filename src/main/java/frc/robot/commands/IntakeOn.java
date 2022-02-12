package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOn extends CommandBase {

    private IntakeSubsystem i_motor1;
    private boolean checkState = true;
    public IntakeOn(IntakeSubsystem intakeMotor1){
        i_motor1 = intakeMotor1;
        addRequirements(i_motor1);
    }

    @Override 
    public void execute() {
        i_motor1.run();
        checkState = false;
    }

    @Override
    public boolean isFinished() {
        return !checkState;
    }

    @Override
    public void end(boolean interrupted) {}
}
    

