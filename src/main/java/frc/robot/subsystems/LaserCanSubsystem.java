package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCanSubsystem extends SubsystemBase {
    private final LaserCan lc;

    private static final double MAX_VALID_DIST = 30.0; // e.g., 100 mm (5 cm)
    private String detectedObj = "nothing";
    private double distance = 0;

    public LaserCanSubsystem() {
        lc = new LaserCan(31);
        try {
            // Configure LaserCan settings
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            detectedObj = "[ERROR] LaserCan configuration failed: " + e.getMessage();
        }
    }

    public boolean processMeasurement() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            distance = measurement.distance_mm;
            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
                    measurement.distance_mm > 0 &&
                    measurement.distance_mm < MAX_VALID_DIST) {
                detectedObj = "Object detected";
                return true;
            } else {
                detectedObj = "No valid object";
                return false;
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("LaserCAN/Game Piece: ", detectedObj);
        SmartDashboard.putString("LaserCAN/Distance: ", String.valueOf(distance));
        processMeasurement();
    }
}