package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {
    private final SparkMax hoodMotor = new SparkMax(8, MotorType.kBrushless);

    private final SmartMotorControllerConfig hoodMotorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            HoodConstants.PROPORTIONAL,
                            HoodConstants.INTEGRAL,
                            HoodConstants.DERIVATIVE,
                            HoodConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            HoodConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("Hood", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(HoodConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(HoodConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    HoodConstants.FEED_FORWARD_KS,
                                    HoodConstants.FEED_FORWARD_KV,
                                    HoodConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    HoodConstants.SIM_FEED_FORWARD_KS,
                                    HoodConstants.SIM_FEED_FORWARD_KV,
                                    HoodConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController hoodSMC =
            new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), hoodMotorConfig);

    private final ArmConfig hoodConfig =
            new ArmConfig(hoodSMC)
                    .withTelemetry("HoodMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimits(HoodConstants.BOTTOM_SOFT_LIMIT, HoodConstants.TOP_SOFT_LIMIT)
                    .withHardLimit(Degrees.of(0), Degrees.of(120))
                    .withLength(HoodConstants.HOOD_LENGTH)
                    .withMass(HoodConstants.HOOD_WEIGHT)
                    .withStartingPosition(
                            HoodConstants
                                    .START_ANGLE); // The Hood can be modeled as an arm since it has
    // a gravitational force acted upon based on the angle its in

    private final Arm hood = new Arm(hoodConfig);

    private boolean isReady() {
        return hood.getAngle()
                .isNear(
                        hood.getMechanismSetpoint().orElse(Degrees.of(0)),
                        HoodConstants.READY_TOLERANCE);
    }

    public Hood() {}

    public final Trigger isReady =
            new Trigger(this::isReady)
                    .debounce(HoodConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);

    public void Update() {}

    public Command setAngle(Supplier<Rotation2d> hoodAngle) {
        return hood.setAngle(Degrees.of(hoodAngle.get().getDegrees()));
    }

    public Command sysId() {
        return hood.sysId(
                Volts.of(4.0), // maximumVoltage
                Volts.per(Second).of(0.5), // step
                Seconds.of(8.0) // duration
                );
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
