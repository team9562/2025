package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class ArmConstants {
    public static final int A_PITCH_ID = 30;
    public static final int A_OPEN_ID = 29;

    public static final int PITCH_STALL_LIMIT = 20;
    public static final int OPEN_STALL_LIMIT = 40;

    public static final double pConversionFactor = 11.66666666666666666667; // converts to degrees with a 1:100 gear reduction ratio
    public static final double oConversionFactor = 0;

    public static final double kP_PITCH = 1.0;
    public static final double kI_PITCH = 0;
    public static final double kD_PITCH = 0;
    public static final double kF_PITCH = 0.01;

    public static final double kP_OPEN = 5.0;
    public static final double kI_OPEN = 0;
    public static final double kD_OPEN = 0;
    public static final double kF_OPEN = 0.01;

    public static final ClosedLoopSlot ARM_SLOT = ClosedLoopSlot.kSlot0;

    public static final double A_TOLERANCE = 1;
}
