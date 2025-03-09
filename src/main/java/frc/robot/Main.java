// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.guzPath;

public final class Main {
  public static final int john = 0;
  public static ArrayList<PVector> botPath = new ArrayList<PVector>();

  private Main() {
  }

  public static void main(String... args) {
    guzPath guzBot = new guzPath();
    System.out.println("Staringg guz thread");
    guzBot.start();
    System.out.println("Thread started successfully.");
    RobotBase.startRobot(Robot::new);
  }
}
