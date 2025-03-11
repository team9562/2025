package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Utility {
    public static boolean withinTolerance(double currentPose, double target, double tolerance) {
        return Math.abs(currentPose - target) <= tolerance;
    }

    public static boolean betweenRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    public static String getLatestXInput(CommandXboxController xbox){
        //IDK HOW ELSE TO DO THIS, FIX IF YOU'RE SMART PLEASE
        if(xbox.a().getAsBoolean()) return "a";
        if(xbox.b().getAsBoolean()) return "b";
        if(xbox.x().getAsBoolean()) return "x";
        if(xbox.y().getAsBoolean()) return "y";
        if(xbox.leftBumper().getAsBoolean()) return "lb";
        if(xbox.rightBumper().getAsBoolean()) return "rb";
        if(xbox.leftTrigger().getAsBoolean()) return "lt";
        if(xbox.rightTrigger().getAsBoolean()) return "rt";
        else return "no input";
    }
}