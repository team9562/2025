# Team 9562 FRC REEFSCAPE 2025 🌊🌊🌊
### Bishop Reding's FRC Reefscape Robot Repo
![Java](https://img.shields.io/badge/Made%20with-Java-blue.svg)

![Royal Robotics](https://avatars.githubusercontent.com/u/160664591?s=200&v=4)

---

## 🔧 **Subsystems**
- 🏎️ **Swerve Drivebase**  
- 📏 **Elevator** *(Three-Stage Lift)*  
- 🪸 **Manipulator/Arm** *(For Coral & Algae Handling)*  
- 🌱 **Algae Ground Intake**  
- 🔥 **LaserCan** *(Mounted on Arm for Object Detection)*  
- 🎯 **Vision System** *(Using Arducam for AprilTag Tracking)*  

---

## ⚙️ **Specs**
### 🏎 **Swerve Drive**
- **SDS MK4i Swerve Modules**
    - *L2 Gear Ratio*
    - *CANCoder for Steering Feedback*
- **CTRE Kraken X60 Motors**
- **TalonFX Motor Controllers**

### ⚡ **Other Subsystems**
- **NEO V1.1 Motors**
- **SparkMax Controllers**
- **LaserCan**
- **Servo (Hang Brake)**

---

## 📚 **Libraries Used**
- **CTRE-Phoenix 6** *(Motor Control)*  
- **Grapple Lib** *(LaserCan Integration)*  
- **Photon Lib** *(Vision Processing with AprilTags)*  
- **Rev Lib** *(SparkMax Control)*  
- **MapleSim** *(Physics Simulation & Kinematics)*  
- **Pathplanner Lib** *(Path Planning)*  

---

## ✨ **Features** ✨
🔹 **AprilTag Position Estimation** *(with PhotonVision & SwerveDrivePoseEstimator)*  
🔹 **Object Detection In Manipulator** *(LaserCan)*  
🔹 **Swerve Odometry** *(Accurate field positioning)*  
🔹 **Cool LED Lights** *(For visualization & team branding!)* 

### **Planned**
🔹**Automatic Pathfinding Via Macros** *(GuzPath™ can find the nearest POI and map)*
🔹**LaserCan Object Seperation Via Vision** *(Determine whether a Coral or Algae is in the Manipulator)*

---

## Naming Conventions
//TODO