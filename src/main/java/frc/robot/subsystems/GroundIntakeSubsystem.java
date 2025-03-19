package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GroundIntakeConstants;
import frc.robot.constants.NeoMotorConstants;
import frc.robot.constants.GroundIntakeConstants.GroundIntakeSetpoint;
import frc.robot.utils.Utility;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeSubsystem extends SubsystemBase {

  // --------- PICKUP MOTOR (for ball pickup) ---------
  private final SparkMax pickupMotor = new SparkMax(GroundIntakeConstants.PICKUP_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig pickupConfig = new SparkMaxConfig();

  // --------- ROTATION MOTORS (for rotating the intake arm) ---------
  // One master and two followers for a total of 3 motors
  private final SparkMax rotateMaster = new SparkMax(GroundIntakeConstants.ROTATE_MASTER_ID, MotorType.kBrushless); // 32
  private final SparkMax rotateFollower = new SparkMax(GroundIntakeConstants.ROTATE_FOLLOWER1_ID, MotorType.kBrushless); // 27

  private final SparkMaxConfig rotateMasterConfig = new SparkMaxConfig();
  private final SparkMaxConfig rotateFollowerConfig = new SparkMaxConfig();

  // PID Controller and encoder for the rotation (using master)
  private final SparkClosedLoopController rotationPID = rotateMaster.getClosedLoopController();
  private final RelativeEncoder rotationEncoder = rotateMaster.getEncoder();

  // Tolerance and target for the rotation (using PID)
  private final double rotationTolerance = GroundIntakeConstants.ROTATION_TOLERANCE;
  private double rotationTarget;
  public static final ClosedLoopSlot slot0 = GroundIntakeConstants.ROTATION_SLOT;

  public GroundIntakeSubsystem() {

    pickupConfig
        .smartCurrentLimit(
            GroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT,
            NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    rotateMasterConfig
        .smartCurrentLimit(
            GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT,
            NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    rotateMasterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            GroundIntakeConstants.ROTATION_kP,
            GroundIntakeConstants.ROTATION_kI,
            GroundIntakeConstants.ROTATION_kD,
            GroundIntakeConstants.ROTATION_kFF,
            slot0);

    rotateMasterConfig.encoder
        .positionConversionFactor(GroundIntakeConstants.ROTATION_POSITION_CONVERSION);

    rotateFollowerConfig
        .smartCurrentLimit(
            GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT,
            NeoMotorConstants.NEO_FREE_LIMIT)
        .voltageCompensation(NeoMotorConstants.NEO_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    rotateFollowerConfig.follow(GroundIntakeConstants.ROTATE_MASTER_ID, false);
  }

  public void burnFlash() {
    pickupMotor.configure(pickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMaster.configure(rotateMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateFollower.configure(rotateFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake(double speed) {
    pickupMotor.set(speed);
  }

  public void stopIntake() {
    pickupMotor.stopMotor();
  }


  public Command setIntakePosition(double position) {
    this.rotationTarget = position;
    return run(() -> rotationPID.setReference(position, ControlType.kPosition, slot0));
    // 0 is upright, 90 in robot parallel, -90 out robot parallel
  }

  public Command setIntakePosition(GroundIntakeSetpoint position) {
    this.rotationTarget = position.getPosition();
    return run(() -> rotationPID.setReference(position.getPosition(), ControlType.kPosition, slot0));
    // 0 is upright, 90 in robot parallel, -90 out robot parallel
  }

  // Get the current intake position (angle)
  public double getIntakePosition() {
    return rotationEncoder.getPosition();
  }

  // Check if the intake is at the target position (within tolerance)
  public boolean isAtTarget() {
    return Utility.withinTolerance(getIntakePosition(), rotationTarget, rotationTolerance);
  }

  public Command runCurrentZeroing() {
    return run(() -> rotationPID.setReference(-2.5, ControlType.kVoltage, slot0))
        .until(() -> isRotationJammed())
        .andThen(() -> rotationEncoder.setPosition(90))
        .finallyDo(() -> rotationPID.setReference(0, ControlType.kVoltage, slot0));
  }

  // Check if the rotation is jammed
  public boolean isRotationJammed() {
    return rotateMaster.getOutputCurrent() > GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT;
  }

  // Reset the intake encoder (set position to 0)
  public void resetIntakeEncoder() {
    rotationEncoder.setPosition(0);
  }

  // Stop the rotation motor
  public void stopRotation() {
    rotateMaster.stopMotor();
  }

  // For testing and debugging.
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pickup Motor Current", pickupMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Rotate Current", rotateMaster.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Arm Position", getIntakePosition());
    SmartDashboard.putBoolean("Intake/Arm At Target", isAtTarget());
    SmartDashboard.putBoolean("Intake/Rotation Jammed", isRotationJammed());
  }
}
