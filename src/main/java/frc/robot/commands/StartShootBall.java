package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSystem;

public class StartShootBall extends CommandBase {
    private ShooterSubSystem shootMotor;

    public StartShootBall(ShooterSubSystem motor) {
        shootMotor = motor;
        addRequirements(shootMotor);
    }

    @Override
    public void execute() {
        shootMotor.run();
    }

}
