// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Main;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.PVector;
import frc.robot.RobotContainer;

public class followGuzPath extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  FieldCentricFacingAngle drive = new FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  CommandJoystick eggYoke;

  double velX = 0, velY = 0;
  double deadZone = 0.5; // 0.5 units actual NOT tile index numbers like 4,7 etc

  int i;
  PVector target = new PVector(-1, -1, -1);
  PVector current = new PVector(0, 0, 0);
  static double unitScale = 1; // Guess and check until test path gives 1 square meter sized path in REAL life
  static double grid[][] = new double[][] { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 },
      { 0.5, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0 },
      { 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.5, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.5 },
      { 0.5, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.5 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5 },
      { 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  public static PVector[] poi = new PVector[22]; // index# + 1 = ID on the map, so POI[0] is id# 1 on the maps
  static int[][] DIRECTIONS = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

  public followGuzPath(CommandSwerveDrivetrain drivesub, CommandJoystick yoke) {
    this.eggYoke = yoke;
    this.m_drivetrain = drivesub;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(">> STARTING");

    // Assign the grid & POI values here

    // To turn the xy inches to meter *0.0254
    poi[0] = new PVector(656.98 * 0.0254, 24.73 * 0.0254, 300);
    poi[1] = new PVector(656.98 * 0.0254, 291.90 * 0.0254, 60);
    poi[2] = new PVector(452.4 * 0.0254, 316.21 * 0.0254, 90);
    poi[3] = new PVector(365.2 * 0.0254, 241.44 * 0.0254, 180);
    poi[4] = new PVector(365.2 * 0.0254, 75.19 * 0.0254, 180);
    poi[5] = new PVector(530.49 * 0.0254, 129.97 * 0.0254, 120);
    poi[6] = new PVector(546.87 * 0.0254, 158.30 * 0.0254, 180);
    poi[7] = new PVector(530.49 * 0.0254, 186.63 * 0.0254, 240);
    poi[8] = new PVector(497.77 * 0.0254, 186.63 * 0.0254, 300);
    poi[9] = new PVector(481.39 * 0.0254, 158.3 * 0.0254, 0);
    poi[10] = new PVector(497.77 * 0.0254, 129.97 * 0.0254, 60);
    poi[11] = new PVector(33.91 * 0.0254, 24.73 * 0.0254, 244);
    poi[12] = new PVector(33.91 * 0.0254, 291.44 * 0.0254, 120);
    poi[13] = new PVector(325.68 * 0.0254, 241.44 * 0.0254, 0);
    poi[14] = new PVector(325.68 * 0.0254, 75.19 * 0.0254, 0);
    poi[15] = new PVector(238.49 * 0.0254, 0.42 * 0.0254, 270);
    poi[16] = new PVector(160.39 * 0.0254, 129.97 * 0.0254, 60);
    poi[17] = new PVector(144 * 0.0254, 158.3 * 0.0254, 0);
    poi[18] = new PVector(160.37 * 0.0254, 186.63 * 0.0254, 300);
    poi[19] = new PVector(193.1 * 0.0254, 186.63 * 0.0254, 240);
    poi[20] = new PVector(209.49 * 0.0254, 158.3 * 0.0254, 180);
    poi[21] = new PVector(193.1 * 0.0254, 129.97 * 0.0254, 120);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.withVelocityX(1).withVelocityY(1);
    // System.out.println(">>" + Main.botPath.size());
    // System.out.println("i: " + i);
    // i++;

    if (Math.abs(eggYoke.getX()) > 0.2 || Math.abs(eggYoke.getY()) > 0.2) {
      this.cancel();
    } else if (this.target.z != -1) {// Spin. End o path below assigns the spin value to home to

      // Here is where Rotation code goes.
      // target.z is the angle you want to get to
      // spin until the current position/orientation is a match
      // Easy!
      m_drivetrain.setControl(drive.withTargetDirection(new Rotation2d(target.z)));

    } else if (this.target.x != -1) {// Homing to positioncode
      // Get current position, target position is tile coordinates
      // Need offsets for the 0.5 meter map gridScale size thats 1 ROBOT in size
      // (1m^2)

      this.velX = 0;
      this.velY = 0;

      // Upgrade this to handle the PID-esque madness to smooth this out
      if (current.x < target.x - deadZone)
        velX = 0.1;
      else if (current.x > target.x + deadZone)
        velX = -0.1;

      if (current.y < target.y - deadZone)
        velY = 0.1;
      else if (current.y > target.y + deadZone)
        velY = -0.1;

      // Maximums
      if (velX > 1)
        velX = 1;
      else if (velX < -1)
        velX = -1;

      if (velY > 1)
        velY = 1;
      else if (velY < -1)
        velY = -1;
      m_drivetrain.setControl(
          drive.withVelocityX(this.velX * RobotContainer.MaxSpeed).withVelocityY(this.velY * RobotContainer.MaxSpeed));

      // For testing purpose, the botPath progression system
      /*
       * if(i > 0)i--;
       * else{
       * current.x = target.x;
       * current.y = target.y;
       * // System.out.println("<Fake Reached: " + current.toString() +"> " +(
       * deadZone*2));
       * i = 200;
       * }
       */

      if (current.dist2D(target) < deadZone * 2) {
        // System.out.println("Node Reached.  Moving On...");
        Main.botPath.remove(0);
        if (Main.botPath.size() > 0) {
          // System.out.println("Next Node available " + Main.botPath.size() + " Remaining... Engaging");
          this.target = Main.botPath.get(0);
          this.target.x = (this.target.x + 0.5) * unitScale;
          this.target.y = (this.target.y + 0.5) * unitScale;
          i = 200;
        } else {
          target.x = -1;
          // System.out.println("End of Path Reached.");
          velX = 0;
          velY = 0;
          // System.out.println("Path System has no Path to follow. STOPPING the Bot!");
          m_drivetrain.setControl(drive.withVelocityX(this.velX * RobotContainer.MaxSpeed)
              .withVelocityY(this.velY * RobotContainer.MaxSpeed));

          // Oreintation to POI happens
          double bestD = current.dist2D(poi[0]);
          int bestPoi = 0;
          for (int i = 0; i < poi.length; i++) {
            if (current.dist2D(poi[i]) < bestD) {
              bestD = current.dist2D(poi[i]);
              bestPoi = i;
            }

            if (bestD < 1.5) {
              target.z = poi[bestPoi].z;
            }

          }

        }
      }

    } else {// online but with no target
      // System.out.println("Assigning TARGET" + Main.botPath.size());
      if (Main.botPath.size() > 0) {
        this.target = Main.botPath.get(0);
        this.target.x = (this.target.x + 0.5) * unitScale;
        this.target.y = (this.target.y + 0.5) * unitScale;
        i++;
      } else {
        this.target.x = -1;// Signal no target

        velX = 0;
        velY = 0;
        // System.out.println("Path System has no Path to follow. STOPPING the Bot!");
        m_drivetrain.setControl(drive.withVelocityX(this.velX * RobotContainer.MaxSpeed)
            .withVelocityY(this.velY * RobotContainer.MaxSpeed));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("ENDING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Main.botPath.size() == 0;
  }

  /// Down here is the path function we need

  public void setCurrentPosition(double x, double y) {
    current.x = x;
    current.y = y;
  }

  public void setCurrentPosition(double x, double y, double o) {
    current.x = x;
    current.y = y;
    current.z = o; // This is angular orientation
  }

  private static class GuzNode {
    int x, y;
    double g, h;
    GuzNode parent;

    GuzNode(int x, int y, double g, double h, GuzNode parent) {
      this.x = x;
      this.y = y;
      this.g = g;
      this.h = h;
      this.parent = parent;
    }

    double f() {
      return g + h;
    }
  }

  public static PVector[] findPath(int startx, int starty, int destx, int desty) {
    int rows = grid.length;
    int cols = grid[0].length;

    PriorityQueue<GuzNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(GuzNode::f));
    boolean[][] closedSet = new boolean[rows][cols];

    openSet.add(new GuzNode(startx, starty, 0, heuristic(startx, starty, destx, desty), null));

    while (!openSet.isEmpty()) {
      GuzNode current = openSet.poll();

      if (current.x == destx && current.y == desty) {
        return reconstructPath(current);
      }

      closedSet[current.x][current.y] = true;

      for (int[] dir : DIRECTIONS) {
        int nx = current.x + dir[0];
        int ny = current.y + dir[1];

        if (nx < 0 || ny < 0 || nx >= rows || ny >= cols || grid[nx][ny] == 1 || closedSet[nx][ny]) {
          continue;
        }

        double g = current.g + ((dir[0] == 0 || dir[1] == 0) ? 1 : Math.sqrt(2));

        double h = heuristic(nx, ny, destx, desty);
        if (grid[nx][ny] != 0)
          h = h / grid[nx][ny];
        openSet.add(new GuzNode(nx, ny, g, h, current));
      }
    }

    return new PVector[0]; // No path found
  }

  static double heuristic(int x1, int y1, int x2, int y2) {
    return Math.abs(x1 - x2) + Math.abs(y1 - y2); // Manhattan Distance
  }

  static PVector[] reconstructPath(GuzNode node) {
    List<PVector> path = new ArrayList<>();
    while (node != null) {
      path.add(new PVector(node.x, node.y, 0));
      node = node.parent;
    }
    Collections.reverse(path);
    return path.toArray(new PVector[0]);
  }

}
