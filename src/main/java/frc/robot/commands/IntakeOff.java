package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOff extends CommandBase{
    private IntakeSubsystem i_motor1;
    private boolean checkState = true;

    public IntakeOff(IntakeSubsystem intakeMotor1){
        i_motor1 = intakeMotor1;
        addRequirements(i_motor1);
    }

    @Override
    public boolean isFinished() {
        return !checkState;
    }

    @Override 
    public void execute(){
        i_motor1.stop();
        checkState = false;
    }
    @Override
    public void end(boolean interrupted) {}
    }