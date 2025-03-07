// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {

    // refer to the google sheets camera coordinate frames:
    // https://docs.google.com/spreadsheets/d/1hgVLrLa4U4E_CwgnRnyZ7TmENQ4SoZw2JJrL0kcaZ2w/edit?gid=0#gid=0

    public static final String cameraName1 = "Arducam1"; // Camera TOP (HOPPER SIDE)
    public static final Transform3d camera1ToRobot = new Transform3d(
            new Translation3d(-5.125, 5.112, 40.04),
            new Rotation3d(0, 60, 180));

    public static final String cameraName2 = "Arducam2"; // Camera TOP (INTAKE SIDE)
    public static final Transform3d camera2ToRobot = new Transform3d(
            new Translation3d(13.375, 5.138, 40.04),
            new Rotation3d(0, 60, 0));

    public static final String cameraName3 = "Arducam3"; // Camera TOP ANGLED RIGHT (INTAKE SIDE)
    public static final Transform3d camera3ToRobot = new Transform3d(
            new Translation3d(13.656, 5.574, 36.753),
            new Rotation3d(0, -40, 26.481));

    public static final String cameraName4 = "Arducam4"; // Camera BOTTOM ANGLED LEFT (INTAKE SIDE)
    public static final Transform3d camera4ToRobot = new Transform3d(
            new Translation3d(13.406, 4.702, 34.678),
            new Rotation3d(0, -40, -26.481));

}
