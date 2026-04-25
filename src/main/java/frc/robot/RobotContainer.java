// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallisticsManager;
import frc.robot.subsystems.BottomFlywheel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.LimelightVisionManager;
import frc.robot.subsystems.PhotonVisionManager;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TargetSelector;
import frc.robot.subsystems.TopFlywheel;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed =
            0.95 // TODO: reset to one
                    * TunerConstants.kSpeedAt12Volts.in(
                            MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(1)
                    .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Create instances of subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Spindexer spindexer = new Spindexer();
    private final Feed feed = new Feed();
    private final IntakeArm intakeArm = new IntakeArm();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final Turret turret = new Turret();
    private final LimelightVisionManager limelightVision =
            new LimelightVisionManager(drivetrain, turret::getAngle, turret::getAngularVelocity);
    private final Lights lights = new Lights();

    public final TargetSelector targetSelector =
            new TargetSelector(() -> drivetrain.getState().Pose);

    private final BallisticsManager ballisticsManager =
            new BallisticsManager(
                    targetSelector.getCurrentTarget(),
                    () -> drivetrain.getState().Pose,
                    turret::getAngle);

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

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Named commands for autonomous
        NamedCommands.registerCommand("IntakeArmDown", intakeArm.lowerIntakeAndFinish());
        NamedCommands.registerCommand("IntakeArmUp", intakeArm.raiseIntakeAndFinish());
        NamedCommands.registerCommand("Shoot", buildAutoShootCommand());
        NamedCommands.registerCommand("IntakeRollers", intakeRoller.spinForwards());
        NamedCommands.registerCommand("ReverseFeed", spindexer.reverse().alongWith(feed.reverse()));
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        bottomFlywheel.setDefaultCommand(bottomFlywheel.idleFlywheel());
        topFlywheel.setDefaultCommand(topFlywheel.idleFlywheel());
        hood.setDefaultCommand(hood.lowerHood());
        turret.setDefaultCommand(turret.idleTurret());

        configureBindings();

        lights.initialize();

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

        joystick.leftBumper().onTrue(intakeArm.toggleIntake());
        joystick.rightTrigger().whileTrue(buildShootCommand());
        joystick.leftTrigger()
                .whileTrue(intakeRoller.spinForwards().alongWith(intakeArm.lowerIntake()));
        joystick.rightBumper()
                .whileTrue(intakeRoller.spinBackwards().alongWith(intakeArm.lowerIntake()));
        joystick.b().whileTrue(spindexer.reverse().alongWith(feed.reverse()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start()
        //         .and(joystick.y())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start()
        //         .and(joystick.x())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command buildShootCommand() {
        return bottomFlywheel
                .spinFlywheel(ballisticsManager.flywheelVelocitySupplier())
                .alongWith(topFlywheel.spinFlywheel(ballisticsManager.flywheelVelocitySupplier()))
                .alongWith(hood.setAngle(ballisticsManager.hoodAngleSupplier()))
                .alongWith(turret.setAngle(ballisticsManager.TX()))
                .alongWith(
                        Commands.waitUntil(
                                        bottomFlywheel
                                                .isReady
                                                .and(topFlywheel.isReady)
                                                .and(hood.isReady)
                                                .and(turret.isReady))
                                .andThen(spindexer.run().alongWith(feed.run())));
    }

    private Command buildAutoShootCommand() {
        return Commands.deadline(
                        Commands.waitUntil(
                                        bottomFlywheel
                                                .isReady
                                                .and(topFlywheel.isReady)
                                                .and(hood.isReady)
                                                .and(turret.isReady))
                                .withTimeout(2.0)
                                .andThen(spindexer.run().alongWith(feed.run()).withTimeout(1.5)),
                        bottomFlywheel.spinFlywheel(ballisticsManager.flywheelVelocitySupplier()),
                        topFlywheel.spinFlywheel(ballisticsManager.flywheelVelocitySupplier()),
                        hood.setAngle(ballisticsManager.hoodAngleSupplier()),
                        turret.setAngle(ballisticsManager.TX()))
                .andThen(
                        bottomFlywheel
                                .idleFlywheel()
                                .alongWith(topFlywheel.idleFlywheel(), hood.lowerHood())
                                .withTimeout(0.02));
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
        SmartDashboard.putNumber(
                "robotSpeed",
                Math.sqrt(
                        Math.pow(drivetrain.getState().Speeds.vxMetersPerSecond, 2)
                                + Math.pow(drivetrain.getState().Speeds.vyMetersPerSecond, 2)));
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    }

    public void onDisabledExit() {
        limelightVision.enableInternalIMU();
    }
}
