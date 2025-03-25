// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.ArmConstants.ArmAngles;
import frc.robot.constants.ArmConstants.IntakeDirection;
import frc.robot.utils.Utility;

public class ArmSubsystem extends SubsystemBase {

  private final SparkMax pitchSpark = new SparkMax(ArmConstants.A_PITCH_ID, MotorType.kBrushless);
  private final RelativeEncoder pitchEncoder = pitchSpark.getEncoder();
  private final AnalogInput lampreyIn = new AnalogInput(3);
  private final AnalogEncoder lamprey = new AnalogEncoder(lampreyIn);
  private final PWM lampreyPWM = new PWM(8);

  private final SparkMax openSpark = new SparkMax(ArmConstants.A_OPEN_ID, MotorType.kBrushless);

  private final SparkMaxConfig basicConfig = new SparkMaxConfig();
  private final SparkMaxConfig pitchConfig = new SparkMaxConfig();
  private final SparkMaxConfig openConfig = new SparkMaxConfig();
  private final SparkClosedLoopController pidPitch = pitchSpark.getClosedLoopController();
  private final SparkClosedLoopController pidOpen = openSpark.getClosedLoopController();
  private final ClosedLoopSlot slot0 = ArmConstants.ARM_SLOT;

  private double target;

  public ArmSubsystem() {
    basicConfig
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .disableFollowerMode()
        .idleMode(IdleMode.kBrake);

    basicConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .maxMotion
        .allowedClosedLoopError(ArmConstants.A_TOLERANCE, slot0)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_ACC)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot0);

    pitchConfig
        .smartCurrentLimit(ArmConstants.PITCH_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(false)
        
    .encoder
        .positionConversionFactor(ArmConstants.pConversionFactor);

    pitchConfig.closedLoop
        .pidf(
            ArmConstants.kP_PITCH,
            ArmConstants.kI_PITCH,
            ArmConstants.kD_PITCH,
            ArmConstants.kF_PITCH,
            slot0);

    // pitchConfig.absoluteEncoder
    // .positionConversionFactor(ArmConstants.pConversionFactor)
    // .zeroOffset(0); // change to actual value

    openConfig
        .smartCurrentLimit(ArmConstants.OPEN_STALL_LIMIT, NeoMotorConstants.NEO_FREE_LIMIT)
        .inverted(false);

    openConfig.closedLoop
        .pidf(
            ArmConstants.kP_OPEN,
            ArmConstants.kI_OPEN,
            ArmConstants.kD_OPEN,
            ArmConstants.kF_OPEN,
            slot0);
  }

  public double getLampreyPose(){
    return lamprey.get();
  }

  public double getEncoderPose() {
    return pitchEncoder.getPosition();
  }

  public void resetPitch(){
    pitchEncoder.setPosition(0);
  }

  public void resetRelative(){
    double absolute = getLampreyPose();
    if(getEncoderPose() < 0) pitchEncoder.setPosition((360 - absolute) * -1);
    if(getEncoderPose() > 0) pitchEncoder.setPosition(absolute);
  }

  public void burnFlash() {
    pitchSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(basicConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pitchSpark.configure(pitchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    openSpark.configure(openConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getOpenCurrent(){
    return openSpark.getOutputCurrent();
  }

  public Command turnPitchMotor(double angle) {
    this.target = angle;
    return run(() -> pidPitch.setReference(angle, ControlType.kPosition, slot0, 0.0, ArbFFUnits.kVoltage));
  }

  public Command turnPitchMotor(ArmAngles angle) {
    this.target = angle.getAngle();
    return run(() -> pidPitch.setReference(angle.getAngle(), ControlType.kPosition, slot0, 0.0, ArbFFUnits.kVoltage));
  }

  public void manualPitchMotor(double volts) {
    pidPitch.setReference(volts * 2, ControlType.kVoltage, slot0, 0.0, ArbFFUnits.kVoltage);
  }

  public Command intakeOuttake(double intake){
    return run(() -> pidOpen.setReference(intake, ControlType.kDutyCycle, slot0));
  }

  public Command intakeOuttake(IntakeDirection direction) { // ex 1 or 0
    return run(() -> pidOpen.setReference(direction.getPower(), ControlType.kDutyCycle, slot0))
      .withTimeout(2)
      .finallyDo(() -> pitchSpark.stopMotor());
  }

  public Command runCurrentZeroing(){
    return run(() -> pidPitch.setReference(1, ControlType.kVoltage, slot0))
    .until(() -> pitchSpark.getOutputCurrent() > 21)
    .andThen(run(() -> pidPitch.setReference(-1, ControlType.kVoltage, slot0)).withTimeout(0.3))
    .andThen(() -> pitchEncoder.setPosition(0))
    .finallyDo(() -> pitchSpark.stopMotor());
  }

  public boolean isAtTarget(){
    return Utility.withinTolerance(getEncoderPose(), target, ArmConstants.A_TOLERANCE);
  }
  
  public boolean isAtCenter(){
    return Utility.withinTolerance(getEncoderPose(), target, ArmConstants.A_TOLERANCE);
  }

  public boolean isWithinSafeRange(){
    return Utility.withinTolerance(getEncoderPose(), -39, 10);
  }

  public boolean isAtPoint(double point){
    return Utility.withinTolerance(getEncoderPose(), point, ArmConstants.A_TOLERANCE);
  }

  public boolean isAtPoint(ArmAngles angle){
    return Utility.withinTolerance(getEncoderPose(), angle.getAngle(), ArmConstants.A_TOLERANCE);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Arm Encoder: ", getEncoderPose());
    SmartDashboard.putNumber("Arm/Lamprey PWM Reading: ", lampreyPWM.getPosition());
    SmartDashboard.putNumber("Arm/Lamprey PWM Pulse(ms): ", lampreyPWM.getPulseTimeMicroseconds());
    SmartDashboard.putNumber("Arm/Lamprey Analog Reading: ", getLampreyPose());
    SmartDashboard.putNumber("Arm/Lamprey Analog V.: ", lampreyIn.getVoltage());
    SmartDashboard.putNumber("Arm/Lamprey Analog Ave. V.: ", lampreyIn.getVoltage());
    SmartDashboard.putNumber("Arm/Arm Target: ", target);
    SmartDashboard.putBoolean("Arm/At Target", isAtTarget());

    /*if(!Utility.withinTolerance(getEncoderPose(), getLampreyPose(), 2)){
      resetRelative();
    }*/
  }
}