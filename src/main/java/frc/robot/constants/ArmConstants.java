package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;

public class ArmConstants {
    public static final int A_PITCH_ID = 30;
    public static final int A_OPEN_ID = 29;

    public static final int PITCH_STALL_LIMIT = 40;
    public static final int OPEN_STALL_LIMIT = 20;

    public static final double pConversionFactor = 3.6;

    public static final double kP_PITCH = 0.01;
    public static final double kI_PITCH = 0;
    public static final double kD_PITCH = 0;
    public static final double kF_PITCH = 0;

    public static final double kP_OPEN = 5.0;
    public static final double kI_OPEN = 0;
    public static final double kD_OPEN = 0;
    public static final double kF_OPEN = 0;

    public static final ClosedLoopSlot ARM_SLOT = ClosedLoopSlot.kSlot0;

    public static final double A_TOLERANCE = 1.25;

    //In dutycycle percent
    public enum IntakeDirection {
        IN(1.0), 
        OUT(-1.0), 
        STOP(0.0);
    
        private final double power;
    
        IntakeDirection(double power) {
            this.power = power;
        }
    
        public double getPower() {
            return power;
        }
    }

    //in degrees
    public enum ArmAngles {
        L2(-103),
        L3(-103),
        L4(-129.94),
        CORAL(39),
        ZERO(0),
        B(0);
    
        private final double angle;
    
        ArmAngles(double angle) {
            this.angle = angle;
        }
    
        public double getAngle() {
            return angle;
        }
    }
    
     
}
