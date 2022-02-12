package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopShootBall extends CommandBase {
    private Shooter s_neo;

    public StopShootBall(Shooter neo) {
        s_neo = neo;
        addRequirements(s_neo);
    }

    @Override
    public void execute() {
        s_neo.stop();
    }

}
