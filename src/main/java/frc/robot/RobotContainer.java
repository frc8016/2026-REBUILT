// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutonomousClimb;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallisticsManager;
import frc.robot.subsystems.BottomFlywheel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.PhotonVisionManager;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TargetSelector;
import frc.robot.subsystems.TopFlywheel;

public class RobotContainer {
    private double MaxSpeed =
            1.0
                    * TunerConstants.kSpeedAt12Volts.in(
                            MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75)
                    .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Create instances of subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Spindexer spindexer = new Spindexer();
    private final Feed feed = new Feed();
    private final IntakeArm intakeArm = new IntakeArm();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    public final TargetSelector targetSelector =
            new TargetSelector(() -> drivetrain.getState().Pose);
    private final BallisticsManager ballisticsManager =
            new BallisticsManager(targetSelector.getCurrentTarget());
    private final BottomFlywheel bottomFlywheel = new BottomFlywheel();
    private final TopFlywheel topFlywheel = new TopFlywheel();
    private final Hood hood = new Hood();

    private final PhotonVisionManager photonVision = new PhotonVisionManager(drivetrain);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        bottomFlywheel.setDefaultCommand(bottomFlywheel.idleFlywheel());
        topFlywheel.setDefaultCommand(topFlywheel.idleFlywheel());
        intakeArm.setDefaultCommand(intakeArm.raiseIntake());

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -joystick.getLeftY()
                                                        * MaxSpeed) // Drive forward with negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -joystick.getLeftX()
                                                        * MaxSpeed) // Drive left with negative X
                                        // (left)
                                        .withRotationalRate(
                                                -joystick.getRightX()
                                                        * MaxAngularRate) // Drive counterclockwise
                        // with negative X (left)
                        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().onTrue(topFlywheel.sysId());
        joystick.b().whileTrue(AutonomousClimb.pathfindToClimb(() -> drivetrain.getState().Pose));

        joystick.rightTrigger()
                .whileTrue(
                        bottomFlywheel
                                .spinFlywheel(
                                        () ->
                                                MetersPerSecond.of(
                                                        10)) // ballisticsManager.flywheelVelocitySupplier()
                                .alongWith(
                                        topFlywheel.spinFlywheel(
                                                () ->
                                                        MetersPerSecond.of(
                                                                10))) // ballisticsManager.flywheelVelocitySupplier()
                                .alongWith(
                                        hood.setAngle(
                                                () ->
                                                        Rotation2d.fromDegrees(
                                                                90)))); // ballisticsManager.hoodAngleSupplier()

        joystick.rightTrigger()
                .and(bottomFlywheel.isReady)
                .and(topFlywheel.isReady)
                .and(hood.isReady)
                .whileTrue(spindexer.run().alongWith(feed.run())); // TODO: add turret is ready

        joystick.rightBumper().toggleOnTrue(intakeArm.lowerIntake());
        joystick.leftTrigger().whileTrue(intakeRoller.spinForwards());

        joystick.x().whileTrue(intakeRoller.spinBackwards());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start()
                .and(joystick.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start()
                .and(joystick.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void updateSubsystems() {
        photonVision.updateVision();
        targetSelector.updateAlliance();
        targetSelector.updateTargetSelection();
        ballisticsManager.update();
    }
}
