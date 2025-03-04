package frc.robot.constants;

public class NeoMotorConstants {

    public static final int NEO_NOMINAL_VOLTAGE = 12;
    public static final int NEO_FREE_LIMIT = 2; // 1.8A
    public static final int NEO_MAX_RPM = 5600; //5676RPM

    public static final double NEO_MAX_VEL = NEO_MAX_RPM;
    public static final double NEO_MAX_ACC = NEO_MAX_VEL / 60;

}
