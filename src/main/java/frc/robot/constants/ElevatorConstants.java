package frc.robot.constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

public class ElevatorConstants {
    //Change device ID's
    public static final int E_LEFT_ID = 0;
    public static final int E_RIGHT_ID = 1;
    public static final int E_STALL_LIMIT = 7; //40A

    public static final double E_GEAR_RATIO = 1 / 6.666666666666667;
    public static final double E_PULLEY_LENGTH = 180 / 25.4; //mm / 25.4 = in
    public static final double kConversionFactor = E_GEAR_RATIO * E_PULLEY_LENGTH;

    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.01;
    public static final ClosedLoopSlot E_SLOT = ClosedLoopSlot.kSlot0;

    //in
    public int E_MAXHEIGHT;
    public int E_MINHEIGHT;

    public int L1;
    public int L2;
    public int L3;
    public int L4;
}
