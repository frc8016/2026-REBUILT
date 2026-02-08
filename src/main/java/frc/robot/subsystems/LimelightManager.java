package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

public class LimelightManager {
    public LimelightManager() {}

    public double getTX() {
        return LimelightHelpers.getTX("");
    }

    public double getTY() {
        return LimelightHelpers.getTY("");
    }
}
