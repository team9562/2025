package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class LaserCanSubsystem extends SubsystemBase {
  private final LaserCan lc;

  // Store recent valid distance measurements
  private static final int SAMPLES = 5;           // Number of samples to collect
  private static final double SD_THRESHOLD = 5.0;   // Standard deviation threshold for classification
  private static final double MAX_VALID_DIST = 50.0;  // e.g., 50 mm (5 cm)
  private final List<Double> recentDistances = new ArrayList<>();

  public LaserCanSubsystem() {
    lc = new LaserCan(31);
    try {
      // Configure LaserCan settings
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("[ERROR] LaserCan configuration failed: " + e.getMessage());
    }
  }

  @Override
  public void periodic() {
    processMeasurement();
  }
  

  public void detectObject() {
    processMeasurement();
  }

  private void processMeasurement() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && 
        measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        measurement.distance_mm > 0 && 
        measurement.distance_mm < MAX_VALID_DIST) {

      double currentDistance = measurement.distance_mm;
      System.out.println("LaserCAN distance: " + currentDistance + " mm");
      recentDistances.add(currentDistance);
      if (recentDistances.size() > SAMPLES) {
        recentDistances.remove(0);
      }
      if (recentDistances.size() == SAMPLES) {
        classifyObject();
      }
    } else {
      System.out.println("[INFO] No valid object detected or out of range.");
    }
  }
  

  private void classifyObject() {
    double stdDev = computeStandardDeviation(recentDistances);
    if (stdDev < SD_THRESHOLD) {
      System.out.println("[DETECTION] Tube (Coral/PCB)");
    } else {
      System.out.println("[DETECTION] Ball (Algae)");
    }
  }


  private double computeStandardDeviation(List<Double> values) {
    double mean = values.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    double variance = values.stream().mapToDouble(val -> Math.pow(val - mean, 2)).sum() / values.size();
    return Math.sqrt(variance);
  }
}
