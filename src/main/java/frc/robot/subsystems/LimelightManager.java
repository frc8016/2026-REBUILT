package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

public class LimelightManager {
    public LimelightManager() {
        LimelightHelpers.setPipelineIndex("", 0);
    }

    public double getTX() {
        return LimelightHelpers.getTX("");
    }

    public double getTY() {
        return LimelightHelpers.getTY("");
    }
}
