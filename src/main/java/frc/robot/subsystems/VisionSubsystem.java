package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTableInstance tableInstance;
    private NetworkTable visionTable;
    private NetworkTable selectedCameraTable;
    
    private double pitch;
    private double yaw;
    private double distance;

    private boolean isTargeting;

    public VisionSubsystem() {
        tableInstance = NetworkTableInstance.getDefault();
        visionTable = tableInstance.getTable("photonvision");
        selectedCameraTable = visionTable.getSubTable("j5_WebCam_JVCU100");
    }

    public void periodic() {
        yaw = (Double) selectedCameraTable.getEntry("targetYaw").getNumber(-70);
        pitch = (Double) selectedCameraTable.getEntry("targetPitch").getNumber(-70);

        SmartDashboard.putNumber("yaw from subsystem", yaw);
        SmartDashboard.putNumber("pitch from subsystem", pitch);

        distance = getDistance();

        SmartDashboard.putNumber("distance from goal", distance);
    }

    public double getCurrentYaw() {
        return yaw;
    }

    public double getCurrentPitch() {
        return pitch;
    }

    public double getDistance() {
        return (104.0 - 30.5) / Math.tan(Math.toRadians(pitch +  26.5));
    }

    public double getTargetYaw() {
        return 90 - Math.toDegrees(Math.atan(distance / 11.5));
    }

    public void startTargeting() {
        isTargeting = true;
    }

    public void stopTargeting() {
        isTargeting = false;
    }

    public boolean isTargeting() {
        return isTargeting;
    }

}
