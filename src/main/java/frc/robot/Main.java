// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  public static ArrayList<PVector> botPath = new ArrayList<PVector>();

  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
    
    // System.out.println("[ROBOT STARTED] We did something right, that's for sure");
  }
}
