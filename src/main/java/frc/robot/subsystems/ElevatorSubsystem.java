// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.utils.Utility;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new elevator. */
  private SparkMax elevatorLeft = new SparkMax(ElevatorConstants.E_LEFT_ID, MotorType.kBrushless);
  private SparkMax elevatorRight = new SparkMax(ElevatorConstants.E_RIGHT_ID, MotorType.kBrushless);
  private final SparkClosedLoopController pid = elevatorLeft.getClosedLoopController();

  private RelativeEncoder leftEncoder = elevatorLeft.getEncoder();
  private RelativeEncoder rightEncoder = elevatorRight.getEncoder();

  private final SparkMaxConfig leftConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfig = new SparkMaxConfig();

  private static final double kP = ElevatorConstants.kP;
  private static final double kI = ElevatorConstants.kI;
  private static final double kD = ElevatorConstants.kD;
  private static final double kFF = ElevatorConstants.kP;
  private static final ClosedLoopSlot slot0 = ElevatorConstants.E_SLOT;

  private double tolerance = ElevatorConstants.E_TOLERANCE;
  private double target;

  public ElevatorSubsystem() {

    rightConfig
      .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, NeoMotorConstants.NEO_MAX_RPM)
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      .disableFollowerMode()
      .inverted(false);

    rightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kP, kI, kD, kFF, slot0)

    .maxMotion
      .allowedClosedLoopError(ElevatorConstants.E_TOLERANCE, slot0)
      .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC, slot0)
      .maxVelocity(NeoMotorConstants.NEO_MAX_VEL, slot0)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot0);

    rightConfig.encoder
      .positionConversionFactor(ElevatorConstants.kConversionFactor); // mm / 25.4 = in

    leftConfig
      .smartCurrentLimit(ElevatorConstants.E_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT, NeoMotorConstants.NEO_MAX_RPM)
      .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .follow(ElevatorConstants.E_RIGHT_ID, true);

    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kP, kI, kD, kFF, slot0)

      .maxMotion
        .allowedClosedLoopError(ElevatorConstants.E_TOLERANCE, slot0)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC, slot0)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL, slot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot0);

    leftConfig.encoder
      .positionConversionFactor(ElevatorConstants.kConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kConversionFactor / 60);
  }

  public void burnFlash(){
    elevatorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoderPose(){
    return rightEncoder.getPosition();
  }

  public void resetEncoderPose(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void stopElevator(){
    elevatorRight.stopMotor();
    elevatorLeft.stopMotor();
  }

  public Command moveElevator(double speed){
    return this
    .run(() -> elevatorRight.set(speed * ElevatorConstants.E_MAXSPEED))
    .until(() -> getEncoderPose() <= ElevatorConstants.E_MAXHEIGHT - 10 || Utility.betweenRange(getEncoderPose(), 0, 2))
    .finallyDo(() -> elevatorRight.stopMotor());
  }

  public void move(double speed){
    pid.setReference(speed, ControlType.kMAXMotionVelocityControl, slot0);
  }

  public double getError(double targetHeight){
    this.target = targetHeight;
    return targetHeight - getEncoderPose();
  }

  public void setElevatorHeight(double targetHeight){
    this.target = targetHeight;
    pid.setReference(getError(targetHeight), ControlType.kMAXMotionPositionControl, slot0);
  }

  public Command runCurrentZeroing(){
    return this
      .run(()-> elevatorRight.setVoltage(-1)) //decrease??
      .until(()-> elevatorRight.getOutputCurrent() > ElevatorConstants.E_STALL_LIMIT)
      .finallyDo(()-> resetEncoderPose());
  }

  public Boolean isAtTarget(){
    return Utility.withinTolerance(getEncoderPose(), target, tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("height: ", getEncoderPose());
    SmartDashboard.putNumber("current: ", elevatorRight.getOutputCurrent());
    SmartDashboard.putNumber("voltage: ", elevatorRight.getBusVoltage());
    SmartDashboard.putBoolean("Target Reached: ", isAtTarget());
  }
}