// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ElevatorConstants.ElevatorHeights;
import frc.robot.utils.Utility;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new elevator. */
  private SparkMax elevatorLeft = new SparkMax(ElevatorConstants.E_LEFT_ID, MotorType.kBrushless);
  private SparkMax elevatorRight = new SparkMax(ElevatorConstants.E_RIGHT_ID, MotorType.kBrushless);
  private final SparkClosedLoopController pid = elevatorRight.getClosedLoopController();

  private RelativeEncoder leftEncoder = elevatorLeft.getEncoder();
  private RelativeEncoder rightEncoder = elevatorRight.getEncoder();

  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = ElevatorConstants.kP;
  private static final double kI = ElevatorConstants.kI;
  private static final double kD = ElevatorConstants.kD;
  private static final double kFF = ElevatorConstants.kFF;
  private static final ClosedLoopSlot slot0 = ElevatorConstants.E_SLOT;

  private double tolerance = ElevatorConstants.E_TOLERANCE;
  private double target;

  // debug info
  double inputVolts = 0;
  boolean isReacting = false;

  public ElevatorSubsystem() {

    rightConfig
        .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, 10000)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .disableFollowerMode()
        .inverted(true);

    rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .minOutput(ElevatorConstants.minOut)
        .maxOutput(ElevatorConstants.maxOut)
        .pidf(kP, kI, kD, 0, slot0);

    rightConfig.encoder
        .positionConversionFactor(ElevatorConstants.kConversionFactor); // mm / 25.4 = in

    leftConfig
        .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, 10000)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .follow(ElevatorConstants.E_RIGHT_ID, true);

    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .minOutput(ElevatorConstants.minOut)
        .maxOutput(ElevatorConstants.maxOut)
        .pidf(kP, kI, kD, 0, slot0);

    leftConfig.encoder
        .positionConversionFactor(ElevatorConstants.kConversionFactor);
  }

  public void burnFlash() {
    elevatorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoderPose() {
    return rightEncoder.getPosition();
  }

  public void resetEncoderPose() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void stopElevator() {
    elevatorRight.stopMotor();
    elevatorLeft.stopMotor();
  }

  public void moveElevator(double volts) {
    this.isReacting = true;
    this.inputVolts = volts;
    pid.setReference(volts * 5, ControlType.kVoltage, slot0);
  }

  public Command setElevatorHeight(double targetHeight) {
    this.target = targetHeight;
    return run(() -> pid.setReference(targetHeight, ControlType.kPosition, slot0, kFF, ArbFFUnits.kVoltage)); // resolve error
  }

  public Command setElevatorHeight(ElevatorHeights height) {
    this.target = height.getHeight();
    return run(() -> pid.setReference(height.getHeight(), ControlType.kPosition, slot0, kFF, ArbFFUnits.kVoltage));
  }

  public Command runCurrentZeroing() {
    return this
        .run(() -> pid.setReference(-2.5, ControlType.kVoltage, slot0)) // decrease??
        .until(() -> elevatorRight.getOutputCurrent() > ElevatorConstants.E_STALL_LIMIT + 1)
        .andThen(() -> pid.setReference(0, ControlType.kVoltage, slot0))
        .finallyDo(() -> rightEncoder.setPosition(0));
  }

  public Command rocketShip(){
    return run(() -> pid.setReference(ElevatorHeights.ZERO.getHeight(), ControlType.kPosition, slot0))
    .until(() -> isAtPoint(ElevatorHeights.ZERO))
    .andThen(runCurrentZeroing());
  }

  public Boolean isAtTarget() {
    return Utility.withinTolerance(getEncoderPose(), target, tolerance);
  }

  public boolean isAtPoint(double point){
    return Utility.withinTolerance(getEncoderPose(), point, ElevatorConstants.E_TOLERANCE);
  }

  public boolean isAtPoint(ElevatorHeights height){
    return Utility.withinTolerance(getEncoderPose(), height.getHeight(), ElevatorConstants.E_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Height: ", getEncoderPose());

    SmartDashboard.putNumber("Elevator/Current: ", elevatorRight.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/Voltage: ", elevatorRight.getBusVoltage());

    SmartDashboard.putBoolean("Elevator/Target Reached: ", isAtTarget());
    SmartDashboard.putNumber("Elevator/Target Height: ", target);

    SmartDashboard.putBoolean("Elevator/Manual Move: ", isReacting);
    SmartDashboard.putNumber("Elevator/Input Volts", inputVolts); // should be between one or negative one
  }
}