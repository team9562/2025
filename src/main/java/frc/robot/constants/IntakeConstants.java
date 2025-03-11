package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class IntakeConstants {

    // Motor IDs for your two intake motors
    public static final int INTAKE_LEFT_ID  = 27;  
    public static final int INTAKE_RIGHT_ID = 28;

    public static final int I_STALL_LIMIT = 20;
    public static final ClosedLoopSlot INTAKE_SLOT = ClosedLoopSlot.kSlot0;

    // PID gains (NOT TUNED)
    public static final double kP  = 0.1;
    public static final double kI  = 0.0;
    public static final double kD  = 0.0;
    public static final double kFF = 0.0;

    public static final double INTAKE_TOLERANCE = 0.5;

    // Conversion factors
    public static final double kPositionConversionFactor = 1.0;
    public static final double kVelocityConversionFactor = 1.0 / 60.0; 
}
