package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

public class LimelightManager {
    public LimelightManager() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
    }

    public double getTX() {
        return 0;
    }
}
