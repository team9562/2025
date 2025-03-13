package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GroundIntakeConstants;
import frc.robot.constants.NeoMotorConstants;
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

  // --------- PICKUP MOTOR (for za ball pickup) ---------
  private final SparkMax pickupMotor = new SparkMax(GroundIntakeConstants.PICKUP_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxConfig pickupConfig = new SparkMaxConfig();

  // --------- ROTATION MOTORS (for rotating the intake arm) ---------
  // One master and two followers for a total of 3 motors
  private final SparkMax rotateMaster = new SparkMax(GroundIntakeConstants.ROTATE_MASTER_ID, MotorType.kBrushless);
  private final SparkMax rotateFollower1 = new SparkMax(GroundIntakeConstants.ROTATE_FOLLOWER1_ID,
      MotorType.kBrushless);
  private final SparkMax rotateFollower2 = new SparkMax(GroundIntakeConstants.ROTATE_FOLLOWER2_ID,
      MotorType.kBrushless);

  private final SparkMaxConfig rotateMasterConfig = new SparkMaxConfig();
  private final SparkMaxConfig rotateFollower1Config = new SparkMaxConfig();
  private final SparkMaxConfig rotateFollower2Config = new SparkMaxConfig();

  // PID Controller and encoder for the rotation (using master)
  private final SparkClosedLoopController rotationPID = rotateMaster.getClosedLoopController();
  private final RelativeEncoder rotationEncoder = rotateMaster.getEncoder();

  // Tolerance and target for the rotation (using PID)
  private final double rotationTolerance = GroundIntakeConstants.ROTATION_TOLERANCE;
  private double rotationTarget = 0.0;
  public static final ClosedLoopSlot ROTATION_SLOT = ClosedLoopSlot.kSlot0;

  //Take some time to clean this up and simplify it lol; also don't flash the motors everytime these are created, could break things
  public GroundIntakeSubsystem() {
    // ---------- Configure Pickup Motor ----------
    pickupConfig
        .smartCurrentLimit(
            GroundIntakeConstants.PICKUP_MOTOR_STALL_LIMIT,
            GroundIntakeConstants.PICKUP_MOTOR_FREE_LIMIT)
        .voltageCompensation(GroundIntakeConstants.PICKUP_MOTOR_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    

    // ---------- Configure Rotation Master Motor ----------
    rotateMasterConfig
        .smartCurrentLimit(
            GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT,
            GroundIntakeConstants.ROTATION_MOTOR_FREE_LIMIT)
        .voltageCompensation(GroundIntakeConstants.ROTATION_MOTOR_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    rotateMasterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            GroundIntakeConstants.ROTATION_kP,
            GroundIntakeConstants.ROTATION_kI,
            GroundIntakeConstants.ROTATION_kD,
            GroundIntakeConstants.ROTATION_kFF,
            GroundIntakeConstants.ROTATION_SLOT).maxMotion
        .allowedClosedLoopError(rotationTolerance, GroundIntakeConstants.ROTATION_SLOT)
        .maxAcceleration(NeoMotorConstants.NEO_MAX_VEL, GroundIntakeConstants.ROTATION_SLOT)
        .maxVelocity(NeoMotorConstants.NEO_MAX_VEL, GroundIntakeConstants.ROTATION_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, GroundIntakeConstants.ROTATION_SLOT);
    rotateMasterConfig.encoder
        .positionConversionFactor(GroundIntakeConstants.ROTATION_POSITION_CONVERSION);


    // ---------- Configure Rotation Follower Motors ----------
    // Follower 1 (non-inverted follower)
    rotateFollower1Config
        .smartCurrentLimit(
            GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT,
            GroundIntakeConstants.ROTATION_MOTOR_FREE_LIMIT)
        .voltageCompensation(GroundIntakeConstants.ROTATION_MOTOR_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    // Set follower to follow master (non-inverted)
    rotateFollower1Config.follow(GroundIntakeConstants.ROTATE_MASTER_ID, false);
    

    // Follower 2 (inverted follower)
    rotateFollower2Config
        .smartCurrentLimit(
            GroundIntakeConstants.ROTATION_MOTOR_STALL_LIMIT,
            GroundIntakeConstants.ROTATION_MOTOR_FREE_LIMIT)
        .voltageCompensation(GroundIntakeConstants.ROTATION_MOTOR_NOMINAL_VOLTAGE)
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    // Set follower to follow master, but inverted relative to master output
    rotateFollower2Config.follow(GroundIntakeConstants.ROTATE_MASTER_ID, true);
  }

  public void burnFlash(){
    pickupMotor.configure(pickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMaster.configure(rotateMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateFollower1.configure(rotateFollower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateFollower2.configure(rotateFollower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // --------------- PICKUP METHODS ---------------
  public void runIntake(double speed) {
    pickupMotor.set(speed);
  }

  public void stopIntake() {
    pickupMotor.stopMotor();
  }

  // --------------- ROTATION METHODS ---------------

  // target position in encoder units (or degrees, if conversion has been set
  // already).
  public void setIntakePosition(double position) {
    this.rotationTarget = position;
    rotationPID.setReference(position, ControlType.kMAXMotionPositionControl, GroundIntakeConstants.ROTATION_SLOT);
  }

  public double getIntakePosition() {
    return rotationEncoder.getPosition();
  }

  public boolean isAtTarget() {
    return Utility.withinTolerance(getIntakePosition(), rotationTarget, rotationTolerance);
  }

  // returns true if the summed current exceeds the threshold. --> meaning it is
  // jammed
  public boolean isRotationJammed() {
    double totalCurrent = rotateMaster.getOutputCurrent()
        + rotateFollower1.getOutputCurrent()
        + rotateFollower2.getOutputCurrent();
    return totalCurrent > GroundIntakeConstants.ROTATION_JAM_CURRENT_THRESHOLD;
  }

  public void resetIntakeEncoder() {
    rotationEncoder.setPosition(0);
  }

  public void stopRotation() {

  }

  // For testing and debugging.
  @Override
  public void periodic() {
    // Pickup motor status
    SmartDashboard.putNumber("Pickup Motor Current", pickupMotor.getOutputCurrent());
    // Rotation motors status
    SmartDashboard.putNumber("Rotate Master Current", rotateMaster.getOutputCurrent());
    SmartDashboard.putNumber("Rotate Follower1 Current", rotateFollower1.getOutputCurrent());
    SmartDashboard.putNumber("Rotate Follower2 Current", rotateFollower2.getOutputCurrent());
    // Arm (rotation) position and status
    SmartDashboard.putNumber("Arm Position", getIntakePosition());
    SmartDashboard.putBoolean("Arm At Target", isAtTarget());
    SmartDashboard.putBoolean("Rotation Jammed", isRotationJammed());

    if (isRotationJammed()) {

    }
  }
}