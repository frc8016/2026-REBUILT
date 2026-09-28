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
import frc.robot.Constants.SpindexerConstants;
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

public class Spindexer extends SubsystemBase {

    private final SparkMax SpindexerMotor = new SparkMax(2, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            SpindexerConstants.PROPORTIONAL,
                            SpindexerConstants.INTEGRAL,
                            SpindexerConstants.DERIVATIVE,
                            SpindexerConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            SpindexerConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("SpindexerMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(SpindexerConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(true)
                    .withClosedLoopRampRate(SpindexerConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(SpindexerConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    SpindexerConstants.FEED_FORWARD_KS,
                                    SpindexerConstants.FEED_FORWARD_KV,
                                    SpindexerConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    SpindexerConstants.SIM_FEED_FORWARD_KS,
                                    SpindexerConstants.SIM_FEED_FORWARD_KV,
                                    SpindexerConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor =
            new SparkWrapper(SpindexerMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig SpindexerConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(SpindexerConstants.FLYWHEEL_DIAMETER)
                    .withMass(SpindexerConstants.FLYWHEEL_MASS)
                    .withTelemetry("SpindexerMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimit(
                            SpindexerConstants.SOFT_LIMIT.negate(), SpindexerConstants.SOFT_LIMIT);
    // .withSpeedometerSimulation(RPM.of(7500)); // optional to make graph of velocity not position

    private final FlyWheel Spindexer = new FlyWheel(SpindexerConfig);

    public Command run(Supplier<LinearVelocity> velocity) {
        return this.Spindexer.run(
                () ->
                        SpindexerConfig.getAngularVelocity(
                                MetersPerSecond.of(velocity.get().in(MetersPerSecond))
                                        .minus(SpindexerConstants.FEED_BUFFER)));
    }

    public Command reverse() {
        return this.Spindexer.set(SpindexerConstants.SPINDEXER_REVERSE_SPEED);
    }

    public Command idleFlywheel() {
        return this.Spindexer.set(0);
    }

    @Override
    public void periodic() {
        Spindexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        Spindexer.simIterate();
    }

    public Command sysId() {
        return Spindexer.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
    }
}
