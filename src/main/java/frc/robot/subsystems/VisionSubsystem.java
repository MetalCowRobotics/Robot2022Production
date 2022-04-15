package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTableInstance tableInstance;
    private NetworkTable visionTable;
    private NetworkTable selectedCameraTable;

    private Relay greenLightRelay;
    private Relay.Value currentValue = Relay.Value.kOff;
    
    private double pitch;
    private double yaw;
    private double distance;

    private boolean isTargeting;

    public VisionSubsystem() {
        tableInstance = NetworkTableInstance.getDefault();
        visionTable = tableInstance.getTable("photonvision");
        selectedCameraTable = visionTable.getSubTable("j5_WebCam_JVCU100");
        greenLightRelay = new Relay(Constants.VISION_RELAY_INDEX);
        greenLightRelay.set(Relay.Value.kForward);
    }

    public void periodic() {
        yaw = (Double) selectedCameraTable.getEntry("targetYaw").getNumber(-70);

        double heightScalar = (4 / 32.85) * Math.abs(yaw);

        pitch = (Double) selectedCameraTable.getEntry("targetPitch").getNumber(-70) - heightScalar;

        SmartDashboard.putNumber("yaw from subsystem", yaw);
        SmartDashboard.putNumber("pitch from subsystem", pitch);

        distance = getDistance();

        SmartDashboard.putNumber("distance from goal", distance);

        greenLightRelay.set(currentValue);
    }

    public double getCurrentYaw() {
        return yaw;
    }

    public double getCurrentPitch() {
        return pitch;
    }
// 32.85
    public double getDistance() {
        return (104.0 - 29.5) / Math.tan(Math.toRadians(pitch +  26.5));
    }

    public double getTargetYaw() {
        double target = 90 - Math.toDegrees(Math.atan(distance / 11.5));

        if (yaw < 0) {
            target -= 10;
        }
        
        return target;
    }

    public double getShooterSpeedScalar() {
        return distance / 90;
    } 

    public void startTargeting() {
        isTargeting = true;
        currentValue = Relay.Value.kForward;
    }

    public void stopTargeting() {
        isTargeting = false;
        currentValue = Relay.Value.kOff;
    }

    public boolean isTargeting() {
        return isTargeting;
    }

}
