package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlyWheelConstants;
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

public class Flywheel extends SubsystemBase {

    private final SparkMax flywheelMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax flywheelMotorfollower = new SparkMax(5, MotorType.kBrushless);
    private final Distance flywheelDiameter = Inches.of(4);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            0.1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withFeedforward(new SimpleMotorFeedforward(0.28, 1.52, 0.175))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 1.52, 0.175))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withFollowers(Pair.of(flywheelMotorfollower, true));

    private final SmartMotorController motor =
            new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
                    .withSoftLimit(RPM.of(-5000), RPM.of(5000));
    // .withSpeedometerSimulation(RPM.of(7500)); // optional to make graph of velocity not position

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private boolean isReady() {
        return MathUtil.isNear(
                FlyWheelConstants.SHOOTING_SETPOINT.in(MetersPerSecond),
                flywheel.getSpeed().in(RotationsPerSecond)
                        * flywheelDiameter.times(Math.PI).in(Meters),
                FlyWheelConstants.READY_TOLERANCE);
    }

    public Flywheel() {}

    public Command spinFlywheel() {
        System.out.println("SHOOT");
        return this.flywheel.setSpeed(
                RotationsPerSecond.of(
                        FlyWheelConstants.SHOOTING_SETPOINT.in(MetersPerSecond)
                                / flywheelDiameter.times(Math.PI).in(Meters)));
    }

    public Command idleFlywheel() {
        System.out.println("IDLE");
        return this.flywheel.setSpeed(
                RotationsPerSecond.of(
                        FlyWheelConstants.IDLE_SETPOINT.in(MetersPerSecond)
                                / flywheelDiameter.times(Math.PI).in(Meters)));
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    // Stay ready for short time after to prevent flapping
                    .debounce(FlyWheelConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
