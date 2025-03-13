package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class GroundIntakeConstants {

    // ---- Pickup Motor Constants (for ball pickup) ----
    public static final int PICKUP_MOTOR_ID = 26;

    // Example stall limit – adjust as needed; free limit taken from
    // NeoMotorConstants.
    public static final int PICKUP_MOTOR_STALL_LIMIT = 20;
    public static final int PICKUP_MOTOR_FREE_LIMIT = NeoMotorConstants.NEO_FREE_LIMIT;

    // If your pickup motor is different from the standard NEO motor, set your own
    // max RPM.
    public static final int PICKUP_MOTOR_MAX_RPM = 100;
    public static final double PICKUP_MOTOR_NOMINAL_VOLTAGE = NeoMotorConstants.NEO_NOMINAL_VOLTAGE;

    // ---- Rotation Motor Constants (for rotating the intake arm) ----
    public static final int ROTATE_MASTER_ID = 28;
    public static final int ROTATE_FOLLOWER1_ID = 32;
    public static final int ROTATE_FOLLOWER2_ID = 27;

    // ---- Current Limits ----
    public static final int ROTATION_MOTOR_STALL_LIMIT = 30;
    public static final int ROTATION_MOTOR_FREE_LIMIT = NeoMotorConstants.NEO_FREE_LIMIT;

    public static final int ROTATION_MOTOR_MAX_RPM = 100;
    public static final double ROTATION_MOTOR_NOMINAL_VOLTAGE = NeoMotorConstants.NEO_NOMINAL_VOLTAGE;

    // ---- PID Constants for Rotation (Not tuned yet) ----
    public static final double ROTATION_kP = 0.1;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;
    public static final double ROTATION_kFF = 0.0;

    // Closed-loop slot for the rotation PID controller.
    public static final ClosedLoopSlot ROTATION_SLOT = ClosedLoopSlot.kSlot0;

    // ---- Tolerances and Conversion Factors for Rotation ----
    // Tolerance for determining when the arm has reached its target (in degrees)
    public static final double ROTATION_TOLERANCE = 1.0;
    // Encoder counts to degrees (adjust based on your gear ratio/encoder specs)
    public static final double ROTATION_POSITION_CONVERSION = 21.07; // ticks per motor degree
    public static final double ROTATION_VELOCITY_CONVERSION = 0.0474; // ticks per degree per second

    // ---- Jamming Detection ----
    // If the summed current from all rotation motors exceeds this threshold (in
    // Amps),
    public static final double ROTATION_JAM_CURRENT_THRESHOLD = 40.0;
}