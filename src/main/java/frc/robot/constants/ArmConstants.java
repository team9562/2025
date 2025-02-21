package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class ArmConstants {
    public static final int ARM_ID = 0;

    public static final int ARM_STALL_LIMIT = 5;

    public static final double kConversionFactor = 0;

    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final ClosedLoopSlot ARM_SLOT = ClosedLoopSlot.kSlot0;

    public static final double OpenPose = 0.002;
}
