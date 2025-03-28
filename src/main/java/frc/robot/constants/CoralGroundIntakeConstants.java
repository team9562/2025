package frc.robot.constants;
import com.revrobotics.spark.ClosedLoopSlot;


public class CoralGroundIntakeConstants {
    public static final int FRONT_PICKUP_MOTOR_ID = 28;
    public static final int BACK_PICKUP_MOTOR_ID = 32;
    public static final int ROTATE_MASTER_ID = 27;

    public static final int BEAM_BREAK_SENSOR_ID = 9;

    public static final int ROTATION_TOLERANCE = 1;
    public static final int PICKUP_MOTOR_STALL_LIMIT = 20;
    public static final int ROTATION_MOTOR_STALL_LIMIT = 20;

    public static final float ROTATION_POSITION_CONVERSION = 1/360f*50f;
    public static final int ROTATION_SLOT = 30;

    public static final int ROTATION_kP = 0;
    public static final int ROTATION_kI = 0;
    public static final int ROTATION_kD = 0;
    public static final int ROTATION_kFF = 0;

    public static final ClosedLoopSlot INTAKE_SLOT = ClosedLoopSlot.kSlot0;

}
