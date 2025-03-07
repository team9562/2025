// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {
        //in meters 
        public static final double camera1X = -0.130175; 
        public static final double camera2X = 0.339725; 
        public static final double camera3X = 0.3468624; 
        public static final double camera4X = 0.3405124; 

        public static final double camera1Y = 0.1298448; 
        public static final double camera2Y = 0.1305052; 
        public static final double camera3Y = 0.1415796; 
        public static final double camera4Y = 0.1194308; 

        public static final double camera1Z = 1.017016; 
        public static final double camera2Z = 1.017016; 
        public static final double camera3Z = 0.9335262; 
        public static final double camera4Z = 0.8808212; 

        //in radians 
        public static final double camera1pitch = 1.0472; 
        public static final double camera2pitch = 1.0472; 
        public static final double camera3pitch = -0.698132; 
        public static final double camera4pitch = -0.698132; 

        public static final double camera1yaw = 3.14159; 
        public static final double camera2yaw = 0; 
        public static final double camera3yaw = 0.462180639; 
        public static final double camera4yaw = -0.462180639;


    // refer to the google sheets camera coordinate frames:
    // https://docs.google.com/spreadsheets/d/1hgVLrLa4U4E_CwgnRnyZ7TmENQ4SoZw2JJrL0kcaZ2w/edit?gid=0#gid=0

    public static final String cameraName1 = "Arducam1"; // Camera TOP (HOPPER SIDE)
    public static final Transform3d camera1ToRobot = new Transform3d(
            new Translation3d(camera1X, camera1Y, camera1Z),
            new Rotation3d(0, camera1pitch, camera1yaw));

    public static final String cameraName2 = "Arducam2"; // Camera TOP (INTAKE SIDE)
    public static final Transform3d camera2ToRobot = new Transform3d(
            new Translation3d(camera2X, camera2Y, camera2Z),
            new Rotation3d(0, camera2pitch, camera2yaw));

    public static final String cameraName3 = "Arducam3"; // Camera TOP ANGLED RIGHT (INTAKE SIDE)
    public static final Transform3d camera3ToRobot = new Transform3d(
            new Translation3d(camera3X, camera3Y, camera3Z),
            new Rotation3d(0, camera3pitch, camera3yaw));

    public static final String cameraName4 = "Arducam4"; // Camera BOTTOM ANGLED LEFT (INTAKE SIDE)
    public static final Transform3d camera4ToRobot = new Transform3d(
            new Translation3d(camera4X, camera4Y, camera4Z),
            new Rotation3d(0, camera4pitch, camera4yaw));

}
