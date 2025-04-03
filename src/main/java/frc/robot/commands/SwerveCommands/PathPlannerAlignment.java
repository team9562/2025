package frc.robot.commands.SwerveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

public class PathPlannerAlignment extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private boolean isLeftBranchUsed;
    private double hypopottamus;
    PathPlannerPath path;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    Pose3d robotPosewow = null;

    public PathPlannerAlignment(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision,
            boolean isLeftBranchUsedababa) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.isLeftBranchUsed = isLeftBranchUsedababa;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {        
        // System.out.println("PATHPLANNER - AUTO ALIGN COMMAND STARTED");

        // get the closest tag -> fix this
        if (vision.targeta == null) {
            // System.out.println("[AutoAlign] No target detected, skipping align command.");
        }
        try {
            if (aprilTagFieldLayout.getTagPose(vision.targeta.getFiducialId()).isPresent()) {
                robotPosewow = PhotonUtils.estimateFieldToRobotAprilTag(vision.targeta.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(vision.targeta.getFiducialId()).get(), vision.cameraPositions);
            }
        } catch (Exception e) {
            // System.out.println("[ERORR] tralalero tralala?: " + e);
        }
        try{
        Pose2d targetPose = vision.calculateScoringPose(vision.tagPose, isLeftBranchUsed);
        // calculates the scoring pose using the tag's pose
       // System.out.println("THIS IS THE DEGREESS: " + robotPosewow.getRotation().getAngle() * 180 / Math.PI);
        //
        // hypopottamus = Math // -> dont need this if not using ideal starting state
        //         .sqrt(Math.pow(vision.currentVelocity.vxMetersPerSecond, 2) + Math.pow(vision.currentVelocity.vyMetersPerSecond, 2));
        // System.out.println("PATHPLANNER STARTING PATH CREATION");
        path = new PathPlannerPath( // make an alignment path using the
                // requires waypoints, constraints, idealstartingstate, endstate
                PathPlannerPath.waypointsFromPoses( // make waypoints using the pose 2Ds
                        vision.myPose3d.toPose2d(), // waypoint at the old pose
                        targetPose // waypoint at the starting pose
                ),
                new PathConstraints(2, 2, Math.toRadians(540), Math.toRadians(720)), 
                // previous velocities: 2.0, 2.0, 540, 720
                // physical path constraints -> max velocity + acceleration and max angular
                // velocity + acceleration
                null,

                // Translation2d -> x & y components of the translation
                // then getNorm is the displacement between origin and end of translation
                // heading is the x & y components using sine and cosine
                new GoalEndState(0.0, targetPose.getRotation())
        // end state = final velocity + final pose heading (angle it wanna be at)
        ); // and thats the whole path!

        // System.out.println("PATHPLANNER -- PATH CREATED---------------------");
        // System.out.println("THIS IS THE TO STRING THING: " + path.toString()); // idk just trying anything atp
        path.preventFlipping = true; // prevent flipping the path if on the opposite (red) alliance

        AutoBuilder.followPath(path); // execute the new path
        }
        catch(Exception e){
            // System.out.println("ERROR AT PATHPLANNER AUTO ALIGN: " + e);
        }
        // everythings good so where the freak is the error brah >:(
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("PATHPLANNER THING REACHED THE END WOAHHAHAHAHAH");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return AutoBuilder.followPath(path).isFinished();
    }

}