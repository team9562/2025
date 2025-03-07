// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Scanner;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

//import static edu.wpi.first.units.Units.Meter;
//import static edu.wpi.first.units.Units.MetersPerSecond;
//import static edu.wpi.first.units.Units.Radians;

//import org.ironmaple.simulation.SimulatedArena;
//import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
//import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
//import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.followGuzPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  CommandSwerveDrivetrain m_drivetrain = RobotContainer.drivetrain;
  Pigeon2 gyro = m_drivetrain.getPigeon2();

  SwerveModule m_frontLeft = m_drivetrain.getModule(0);
  SwerveModule m_frontRight = m_drivetrain.getModule(1);
  SwerveModule m_backLeft = m_drivetrain.getModule(2);
  SwerveModule m_backRight = m_drivetrain.getModule(3);

  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    m_drivetrain.getKinematics(), 
    gyro.getRotation2d(), 
    new SwerveModulePosition[]{ 
      m_frontLeft.getPosition(true),
      m_frontRight.getPosition(true),
      m_backLeft.getPosition(true),
      m_backRight.getPosition(true),
    }, new Pose2d(0, 0, new Rotation2d(0)));

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    poseEstimator.update(gyro.getRotation2d(), 
    new SwerveModulePosition[]{ 
      m_frontLeft.getPosition(true),
      m_frontRight.getPosition(true),
      m_backLeft.getPosition(true),
      m_backRight.getPosition(true),
    });

    double currentPosX = poseEstimator.getEstimatedPosition().getX();
    double currentPosY = poseEstimator.getEstimatedPosition().getY();
    CommandScheduler.getInstance().run();

    System.out.println("X: " + currentPosX);
    System.out.println("Y: " + currentPosY);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
