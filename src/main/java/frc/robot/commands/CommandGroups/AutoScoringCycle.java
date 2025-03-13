// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.HomeArm;
import frc.robot.commands.ArmCommands.IntakeOutake;
import frc.robot.commands.ArmCommands.SetAngleToPOI;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoringCycle extends SequentialCommandGroup {
  ElevatorSubsystem m_ElevatorSubsystem;
  ArmSubsystem m_ArmSubsystem;
  String poi;

  /** Creates a new AutoScoringCycle. */
  public AutoScoringCycle(ElevatorSubsystem elevator, ArmSubsystem arm, String POI) {
    this.m_ElevatorSubsystem = elevator;
    this.m_ArmSubsystem = arm;
    this.poi = POI;
    // Creates a sequence that goes as such
    // 1. Sets the Elevator and Arm positions to the desired height
    // 2. Shoots out the coral
    // 3. Sets the arm to neutral position
    // 4. Homes Elevator
    // 5. Primes arm for next coral
    // cannot add the coral intake command as the time between the priming and the
    // time between reaching station is different
    addCommands(new SetHeightAngleToPOI(m_ElevatorSubsystem, m_ArmSubsystem, poi),
        new IntakeOutake(m_ArmSubsystem, "out"), new HomeArm(m_ArmSubsystem),
        new HomeElevator(m_ElevatorSubsystem), new SetAngleToPOI(m_ArmSubsystem, "coral"));
  }
}
