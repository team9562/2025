# REEFSCAPE 2025
## The official Bishop Reding 2025 REEFSCAPE robot repository.
HCDSB

## Subsystems
- Swerve Drivebase
- Elevator (Three Stage)
- Manipulator/Arm (For Coral and Algae)
- Algae Ground Intake
- LaserCan 
- Vision (Arducam)

## Specs
- RoboRIO v2.0

### Swerve
- SDS MK4i Swerve Modules
    - L2 Gear Ratio
    - CANCoder
- CTRE Kraken X60 Motors
- TalonFX Motor Controllers

### Other Subsystems
- NEO V1.1
- SparkMax Controllers
- LaserCan
- Servo (Hang Brake)

### Libraries
- CTRE-Phoenix 6
- Grapple Lib (LaserCan)
- Photon Lib
- Rev Lib
- MapleSim
- Pathplanner Lib

## Features
- AprilTag Position Estimation with Photon Vision
- Object Detection with LaserCan
- Cool LED Lights
- (Planned) PathFinding