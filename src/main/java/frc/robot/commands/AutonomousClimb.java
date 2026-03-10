// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonomousClimbConstants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class AutonomousClimb {

    public static Command pathfindToClimb(Supplier<Pose2d> poseSupplier) {
        return Commands.defer(
                () -> {
                    Map<Pose2d, String> poseMap = getLineupPoseMap();
                    Pose2d closest = poseSupplier.get().nearest(new ArrayList<>(poseMap.keySet()));
                    String pathName = "climb_" + poseMap.get(closest);

                    Optional<PathPlannerPath> maybePath = loadPath(pathName);
                    if (maybePath.isPresent()) {
                        return AutoBuilder.pathfindThenFollowPath(
                                maybePath.get(), AutonomousClimbConstants.constraints);
                    } else {
                        return Commands.none();
                    }
                },
                Set.of());
    }

    private static Map<Pose2d, String> getLineupPoseMap() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return AutonomousClimbConstants.RED_CLIMB_POSITIONS;
        } else {
            return AutonomousClimbConstants.BLUE_CLIMB_POSITIONS;
        }
    }

    private static Optional<PathPlannerPath> loadPath(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(pathName));
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("error in loading path");
            e.printStackTrace();
            return Optional.empty();
        }
    }
}
