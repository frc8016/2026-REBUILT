package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeedConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Feed extends SubsystemBase {

    private final SparkMax FeedMotor = new SparkMax(1, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            FeedConstants.PROPORTIONAL,
                            FeedConstants.INTEGRAL,
                            FeedConstants.DERIVATIVE,
                            FeedConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            FeedConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("FeedMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(FeedConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(FeedConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(FeedConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    FeedConstants.FEED_FORWARD_KS,
                                    FeedConstants.FEED_FORWARD_KV,
                                    FeedConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    FeedConstants.SIM_FEED_FORWARD_KS,
                                    FeedConstants.SIM_FEED_FORWARD_KV,
                                    FeedConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor =
            new SparkWrapper(FeedMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig feedConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(FeedConstants.FLYWHEEL_DIAMETER)
                    .withMass(FeedConstants.FLYWHEEL_MASS)
                    .withTelemetry("FeedMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimit(FeedConstants.SOFT_LIMIT.negate(), FeedConstants.SOFT_LIMIT);
    // .withSpeedometerSimulation(RPM.of(7500)); // optional to make graph of velocity not position

    private final FlyWheel Feed = new FlyWheel(feedConfig);

    public Command run(Supplier<LinearVelocity> velocity) {
        return this.Feed.run(
                () ->
                        feedConfig.getAngularVelocity(
                                MetersPerSecond.of(velocity.get().in(MetersPerSecond))
                                        .minus(FeedConstants.FEED_BUFFER)));
    }

    public Command reverse() {
        return this.Feed.set(FeedConstants.FEED_REVERSE_SPEED);
    }

    public Command idleFlywheel() {
        return this.Feed.set(0);
    }

    @Override
    public void periodic() {
        Feed.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        Feed.simIterate();
    }

    public Command sysId() {
        return Feed.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
    }
}
