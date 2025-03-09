package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class LaserCanSubsystem extends SubsystemBase {
  private LaserCan lc;

  // Store recent valid distance measurements
  private static final int SAMPLES = 5;           // Number of samples to collect
  private static final double SD_THRESHOLD = 5.0; // Standard deviation threshold for classification
  private static final double MAX_VALID_DIST = 2000.0; // e.g., 2000 mm (2 meters)
  private List<Double> recentDistances = new ArrayList<>();

  public LaserCanSubsystem() {
    // Instantiate LaserCan on CAN ID 31
    lc = new LaserCan(31);
    try {
      // Configure LaserCan settings
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    // Read the latest measurement from LaserCan
    LaserCan.Measurement measurement = lc.getMeasurement();

    // Check if we got a valid measurement
    if (measurement != null && 
        measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        measurement.distance_mm > 0 && 
        measurement.distance_mm < MAX_VALID_DIST) {

      double currentDistance = measurement.distance_mm;
      System.out.println("LaserCAN distance: " + currentDistance + " mm");

      // Add the new distance to the recentDistances list
      recentDistances.add(currentDistance);
      // If we have more samples than needed, remove the oldest one
      if (recentDistances.size() > SAMPLES) {
        recentDistances.remove(0);
      }

      // Only classify if we have enough samples
      if (recentDistances.size() == SAMPLES) {
        double stdDev = computeStandardDeviation(recentDistances);
        // Classify based on standard deviation
        if (stdDev < SD_THRESHOLD) {
          System.out.println("Object Detected: Tube (Coral)");
        } else {
          System.out.println("Object Detected: Ball (Algae)");
        }
      }

    } else {
      // Measurement is invalid or out of range
      System.out.println("No valid object detected or out of range.");
    }
  }

  /**
   * Computes the standard deviation of a list of doubles.
   * @param values List of distance measurements
   * @return Standard deviation
   */
  private double computeStandardDeviation(List<Double> values) {
    if (values.isEmpty()) {
      return 0.0;
    }
    double mean = 0.0;
    for (double val : values) {
      mean += val;
    }
    mean /= values.size();

    double sumSquaredDiff = 0.0;
    for (double val : values) {
      sumSquaredDiff += Math.pow(val - mean, 2);
    }
    return Math.sqrt(sumSquaredDiff / values.size());
  }
}
