package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public static class Library {
    public static void pushDashboard(String varName, String varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putString(varName, varValue);
        }
    }

    public static void pushDashboard(string varName, double varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putDouble(varName, varValue);
        }
    }


    public static void pushDashboard(String varName, Boolean varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putBoolean(varName, varValue);
        }
    }
}
