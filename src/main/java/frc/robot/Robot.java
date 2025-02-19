// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // maplesim game piece telemetry via wpilib networktables
  // in here instead of Telemetry.java partially because it was midnight and partially because here we can distinguish between whats a simulation and whats not
  //StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
  //.getStructArrayTopic("CoralPoseArray", Pose3d.struct)
  //.publish();

  //StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
  //.getStructArrayTopic("AlgaePoseArray", Pose3d.struct)
  //.publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
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
  public void simulationInit() {
    //SimulatedArena arena = SimulatedArena.getInstance();
    //arena.resetFieldForAuto(); // spawn default coral and algae stacks
    //arena.addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(3, 3))); // testing
    //arena.addGamePiece(new ReefscapeCoralOnField(new Pose2d(new Translation2d(8, 8), new Rotation2d())));
  }

  @Override
  public void simulationPeriodic() {
    //SimulatedArena arena = SimulatedArena.getInstance();

    //arena.simulationPeriodic(); // update simulation

    // publish game piece data via wpilib networktables for viewing in advantagescope
    //coralPoses.set(arena.getGamePiecesArrayByType("Coral"));
    //algaePoses.set(arena.getGamePiecesArrayByType("Algae"));

    // can't test the robot without a joystick, so i need some way to make sure the sim works ¯\_(ツ)_/¯
    //ReefscapeAlgaeOnFly testAlgae = new ReefscapeAlgaeOnFly(
    //  m_robotContainer.drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(),
    //  new Translation2d(),
    //  m_robotContainer.drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
    //  m_robotContainer.drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getRotation(),
    //  Meter.of(1),
    //  MetersPerSecond.of(3), 
    //  Angle.ofBaseUnits(Math.toRadians(55), Radians)
    //);
    //testAlgae.enableBecomesGamePieceOnFieldAfterTouchGround();
    //arena.addGamePieceProjectile(testAlgae);
  }
}
