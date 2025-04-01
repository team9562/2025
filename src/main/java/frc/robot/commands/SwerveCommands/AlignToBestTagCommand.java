package frc.robot.commands.SwerveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

public class AlignToBestTagCommand extends InstantCommand {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    public AlignToBestTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        PhotonTrackedTarget target = vision.getClosestTarget();
        if (target == null) {
            System.out.println("[AutoAlign] No target detected.");
            return;
        }

        Pose2d tagPose = vision.estimatePose(vision.getOldPose()); // Optional override: use AprilTagFieldLayout
        Pose2d targetPose = vision.calculateScoringPose(tagPose, true);

        ChassisSpeeds currentVelocity = drivetrain.getKinematics().toChassisSpeeds();
        // now i have no clue if this is gonna work, but if it does then thats cool af and ill take it LOL
        Rotation2d heading = new Rotation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(vision.getOldPose().getTranslation(), heading),
                        targetPose
                ),
                new PathConstraints(2.0, 2.0, Math.toRadians(540), Math.toRadians(720)),
                new IdealStartingState(new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond).getNorm(), heading),
                new GoalEndState(0.0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();
    }
}