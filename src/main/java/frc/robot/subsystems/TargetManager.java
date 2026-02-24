package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class TargetManager {
    private Supplier<Pose3d> targetPose3d;

    public TargetManager(Supplier<Pose3d> targetPose) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
    }

    public double getTX() {
        return 0;
    }
}
