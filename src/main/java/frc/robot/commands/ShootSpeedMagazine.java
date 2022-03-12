package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeedMagazine extends CommandBase{
    private ShooterSubsystem m_motor1;
    private MagazineSubsystem m_motor2;
    private boolean checkState = true;
    public ShootSpeedMagazine(ShooterSubsystem motor1, MagazineSubsystem motor2) {
        m_motor1 = motor1;
        m_motor2 = motor2;
        addRequirements(m_motor1, m_motor2);
     }
 
     @Override 
     public void execute() {
         m_motor1.run();
         if(m_motor1.isReady()) {
             m_motor2.run();
             checkState = false;
         } else {
             m_motor2.stop();
         }
         
     }
 
     @Override
     public boolean isFinished() {
         return !checkState;
     }
 
     @Override
     public void end(boolean interrupted) {   
     }
}
    