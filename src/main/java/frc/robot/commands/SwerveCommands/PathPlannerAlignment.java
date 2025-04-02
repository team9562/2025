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
    private boolean isDoneAligning = false;
    private double hypopottamus;
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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        System.out.println("PATHPLANNER - AUTO ALIGN COMMAND STARTED");

        PhotonTrackedTarget target = vision.getClosestTarget();
        // get the closest tag -> fix this
        if (target == null) {
            System.out.println("[AutoAlign] No target detected.");
            return;
        }
        try {
            if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                robotPosewow = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), vision.cameraPositions);
            }
        } catch (Exception e) {
            System.out.println("[ERORR] tralalero tralala?: " + e);
        }
        Pose2d tagPose = vision.estimatePose(vision.getOldPose()); // Optional override: use AprilTagFieldLayout
        Pose2d targetPose = vision.calculateScoringPose(tagPose, isLeftBranchUsed);
        // calculates the scoring pose using the tag's pose
        ChassisSpeeds currentVelocity = drivetrain.getKinematics().toChassisSpeeds(
                drivetrain.getModules()[0].getCurrentState(), drivetrain.getModules()[1].getCurrentState(),
                drivetrain.getModules()[2].getCurrentState(), drivetrain.getModules()[3].getCurrentState()); // drivetrain.getKinematics().toChassisSpeeds()
        // toChassisSpeeds() has parameters: SwerveModuleState, which we dont use
        // (if anything doesnt work its probably because of this)
        System.out.println("THIS IS THE DEGREESS: " + robotPosewow.getRotation().getAngle() * 180 / Math.PI);
        //
        hypopottamus = Math
                .sqrt(Math.pow(currentVelocity.vxMetersPerSecond, 2) + Math.pow(currentVelocity.vyMetersPerSecond, 2));
        System.out.println("PATHPLANNER STARTING PATH CREATION");
        PathPlannerPath path = new PathPlannerPath( // make an alignment path using the
                // requires waypoints, constraints, idealstartingstate, endstate
                PathPlannerPath.waypointsFromPoses( // make waypoints using the pose 2Ds
                        robotPosewow.toPose2d(), // waypoint at the old pose
                        targetPose // waypoint at the starting pose
                ),
                new PathConstraints(2, 2, Math.toRadians(540), Math.toRadians(720)),
                // physical path constraints -> max velocity + acceleration and max angular
                // velocity + acceleration
                // null,
                new IdealStartingState(hypopottamus, robotPosewow.getRotation().toRotation2d()),

                // Translation2d -> x & y components of the translation
                // then getNorm is the displacement between origin and end of translation
                // heading is the x & y components using sine and cosine
                new GoalEndState(0.0, targetPose.getRotation())
        // end state = final velocity + final pose heading (angle it wanna be at)
        ); // and thats the whole path!

        System.out.println("PATHPLANNER -- PATH CREATED---------------------");
        System.out.println("THIS IS THE TO STRING THING: " + path.toString()); // idk just trying anything atp
        path.preventFlipping = true; // prevent flipping the path if on the opposite (red) alliance

        AutoBuilder.followPath(path).schedule(); // execute the new path
        if (AutoBuilder.followPath(path).isFinished()) { // wont end lol
            isDoneAligning = true;
        }
        
        System.out.println("PATHPLANNER THING REACHED THE END WOAHHAHAHAHAH");
        // everythings good so where the freak is the error brah >:(
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDoneAligning;
    }

}