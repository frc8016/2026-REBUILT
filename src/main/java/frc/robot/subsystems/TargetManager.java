package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class TargetManager {
    private Supplier<Pose3d> targetPose;

    public TargetManager(Supplier<Pose3d> targetPose) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
        this.targetPose = targetPose;
    }

    public double computeTX() {
        Pose2d turretPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
        Pose2d target = targetPose.get().toPose2d();
        return target.relativeTo(turretPose).getTranslation().getAngle().getDegrees();
    }

    public Supplier<Double> getTX() {
        return this::computeTX;
    }
}
