package frc.robot.utils;

public class Utility {
    public static boolean withinTolerance(double currentPose, double target, double tolerance) {
        return Math.abs(currentPose - target) <= tolerance;
    }

    public static boolean betweenRange(double value, double min, double max) {
        return value >= min && value <= max;
    }
}