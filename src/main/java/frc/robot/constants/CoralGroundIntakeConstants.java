package frc.robot.constants;
import com.revrobotics.spark.ClosedLoopSlot;


public class CoralGroundIntakeConstants {
    public static final int FRONT_PICKUP_MOTOR_ID = 28;
    public static final int BACK_PICKUP_MOTOR_ID = 32;
    public static final int ROTATE_MASTER_ID = 27;

    public static final int BEAM_BREAK_SENSOR_ID = 9;

    public static final double ROTATION_TOLERANCE = 0.01;
    public static final int PICKUP_MOTOR_STALL_LIMIT = 40;
    public static final int ROTATION_MOTOR_STALL_LIMIT = 40;

    public static final int ROTATION_SLOT = 30;

    public static final double ROTATION_kP = 1.4;
    public static final int ROTATION_kI = 0;
    public static final int ROTATION_kD = 0;

    public static final ClosedLoopSlot INTAKE_SLOT = ClosedLoopSlot.kSlot0;

    public enum CoralAngles{
        FLOOR(0.647),
        CORAL(0.405),
        ZERO(0.002);

        private final double angle;
    
        CoralAngles(double angle) {
            this.angle = angle;
        }
    
        public double getAngle() {
            return angle;
        }
    }
}
