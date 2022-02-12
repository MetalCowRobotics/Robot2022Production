package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSystem;

public class StopShootBall extends CommandBase {
    private ShooterSubSystem shootMotor;

    public StopShootBall(ShooterSubSystem motor) {
        shootMotor = motor;
        addRequirements(shootMotor);
    }

    @Override
    public void execute() {
        shootMotor.stop();
    }

}
