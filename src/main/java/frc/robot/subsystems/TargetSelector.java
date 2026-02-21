package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TargetConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class TargetSelector {
    private Supplier<Pose2d> swervePoseSupplier;
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Pose2d target =
            new Pose2d(); // output target updates every time updateTargetSelection() is called

    public TargetSelector(Supplier<Pose2d> poseSupplierIn) {
        swervePoseSupplier = poseSupplierIn;
    }

    public void updateTargetSelection() { // updates target Pose2d
        Distance x = Meters.of(swervePoseSupplier.get().getX()); // distance x of swerve
        Distance y = Meters.of(swervePoseSupplier.get().getY()); // distance y of swerve
        Pose2d hubTarget = TargetConstants.HUB_TARGET_BLUE;
        Pose2d ferryTargetLower = TargetConstants.FERRY_TARGET_BLUE_LOWER;

        // this flips the blue Pose2ds for red side and flips swerve pose
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            x = TargetConstants.FIELD_LENGTH.minus(x);
            y = TargetConstants.FIELD_HEIGHT.minus(y);
            hubTarget =
                    new Pose2d(
                            TargetConstants.FIELD_LENGTH.in(Meters) - hubTarget.getX(),
                            hubTarget.getY(),
                            new Rotation2d());
            ferryTargetLower =
                    new Pose2d(
                            TargetConstants.FIELD_LENGTH.in(Meters) - ferryTargetLower.getX(),
                            TargetConstants.FIELD_HEIGHT.in(Meters) - ferryTargetLower.getY(),
                            new Rotation2d());
        }

        if (alliance.isPresent()) { // safety to make sure there is an alliance
            if (x.lte(TargetConstants.DRIVERSTATION_TO_TRENCH)) { // if robot is in home area
                target = hubTarget;
            } else { // if robot is anywhere else
                if (y.lte(TargetConstants.FIELD_HEIGHT.div(2))) { // if robot is on bottom of field
                    target = ferryTargetLower;
                } else { // if robot is on top of field
                    double yFlip =
                            TargetConstants.FIELD_HEIGHT.in(Meters) - ferryTargetLower.getY();
                    // this switches the lower Pose2d for the ferry to be upper
                    target = new Pose2d(ferryTargetLower.getX(), yFlip, new Rotation2d());
                }
            }
        }
    }

    public Pose2d getCurrentTarget() {
        return target;
    }

    public void updateAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance();
        }
    }
}
