package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class GroundIntakeConstants {

    // ---- Pickup Motor Constants (for ball pickup) ----
    public static final int PICKUP_MOTOR_ID = 26;

    // Stall and free limits for the pickup motor
    public static final int PICKUP_MOTOR_STALL_LIMIT = 20;

    // ---- Rotation Motor Constants (for rotating the intake arm) ----
    public static final int ROTATE_MASTER_ID = 28;
    public static final int ROTATE_FOLLOWER1_ID = 27;
    public static final int ROTATE_FOLLOWER2_ID = 32;

    // Stall and free limits for the rotation motor
    public static final int ROTATION_MOTOR_STALL_LIMIT = 40;

    // ---- PID Constants for Rotation ----
    public static final double ROTATION_kP = 0.1;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;
    public static final double ROTATION_kFF = 0.0;

    // Closed-loop slot for the rotation PID controller
    public static final ClosedLoopSlot ROTATION_SLOT = ClosedLoopSlot.kSlot0;

    // ---- Tolerances and Conversion Factors for Rotation ----
    public static final double ROTATION_TOLERANCE = 1.0; // Tolerance in degrees
    public static final double ROTATION_POSITION_CONVERSION = 21.07; // Encoder ticks per degree

    // ---- Jamming Detection ----
    public static final double ROTATION_JAM_CURRENT_THRESHOLD = 40.0; // Amperage threshold for jam detection
}
