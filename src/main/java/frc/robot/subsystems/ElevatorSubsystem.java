// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.NeoMotorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new elevator. */
  private SparkMax elevatorLeft = new SparkMax(ElevatorConstants.E_LEFT_ID, MotorType.kBrushless);
  private SparkMax elevatorRight = new SparkMax(ElevatorConstants.E_RIGHT_ID, MotorType.kBrushless);
  private final SparkClosedLoopController pid = elevatorLeft.getClosedLoopController();

  private RelativeEncoder leftEncoder = elevatorLeft.getEncoder();
  private RelativeEncoder rightEncoder = elevatorRight.getEncoder();

  private DigitalInput limitSwitch = new DigitalInput(0);

  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = ElevatorConstants.kP;
  private static final double kI = ElevatorConstants.kI;
  private static final double kD = ElevatorConstants.kD;
  private static final double kFF = ElevatorConstants.kP;
  private static final ClosedLoopSlot slot0 = ElevatorConstants.E_SLOT;
  private double height;

  public ElevatorSubsystem() {

    leftConfig
      .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, NeoMotorConstants.NEO_MAX_RPM)
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      .disableFollowerMode();

    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kP, kI, kD, kFF, slot0);

    
    leftConfig.encoder
      .positionConversionFactor(ElevatorConstants.kConversionFactor); // mm / 25.4 = in
      //.velocityConversionFactor(ElevatorConstants.E_PULLEY_LENGTH * ElevatorConstants.E_GEAR_RATIO);

    rightConfig
      .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, NeoMotorConstants.NEO_MAX_RPM)
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      .follow(ElevatorConstants.E_LEFT_ID/*, true*/); // inverted???
  }

  public void burnFlash(){
    elevatorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //elevatorRight.configure();
  }

  public double getEncoderPose(){
    height = leftEncoder.getPosition();
    return height;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getEncoderPose();
    SmartDashboard.putNumber("height", height);
  }
}