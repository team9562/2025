// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.HomeArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeThenSetToPOI extends SequentialCommandGroup {
  /** Creates a new HomeThenSetToPOI. */
  ArmSubsystem m_arm;
  ElevatorSubsystem m_ele;
  String poi;

  public HomeThenSetToPOI(ArmSubsystem sub1, ElevatorSubsystem sub2, String POI) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = sub1;
    this.m_ele = sub2;
    this.poi = POI;
    addCommands(new HomeArm(m_arm), new SetHeightAngleToPOI(m_ele, m_arm, poi));
  }
}
