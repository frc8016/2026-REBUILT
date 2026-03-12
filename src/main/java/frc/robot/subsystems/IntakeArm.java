package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

public class IntakeArm extends SubsystemBase {

    private final SparkMax armMotorLeft =
            new SparkMax(6, MotorType.kBrushless); // TODO: Might need to fix the motor id
    private final SparkMax armMotorRight =
            new SparkMax(7, MotorType.kBrushless); // TODO: Might need to fix the motor id

    // There are most definatly a lot of constants and other variables that need to be edited to
    // make this code accurate to our mechanical arm.
    // There are some declerations in the constants folder for certain varibles already, but they
    // also need their values changed.

    // Declares a motor configuration.
    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            1, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("intakeMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withFeedforward(new ArmFeedforward(0.03, 0.01, 0, 0.01))
                    .withSimFeedforward(new ArmFeedforward(0, 0.01, 0.01))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withFollowers(Pair.of(armMotorRight, true));

    // Declares a motor using the motor configuration previously developed.
    private final SmartMotorController motor1 =
            new SparkWrapper(armMotorLeft, DCMotor.getNEO(1), motorConfig);

    // Declares an arm configuration.
    private final ArmConfig m_config =
            new ArmConfig(motor1)
                    .withLength(ArmConstants.ARM_LENGTH)
                    .withSoftLimits(Degrees.of(0), Degrees.of(100)) // TODO: fix constant
                    .withHardLimit(Degrees.of(0), Degrees.of(100))
                    .withTelemetry("intakeArmMechanism", TelemetryVerbosity.HIGH)
                    .withMass(ArmConstants.MASS)
                    .withStartingPosition(ArmConstants.START_ANGLE);

    // Declares an arm using the arm configuration.
    private final Arm arm = new Arm(m_config);

    // Declares an system for the arm containing commands and periodics.
    public IntakeArm() {}

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

    public Command sysId() {
        return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }

    public Command lowerIntake() {
        return this.arm.setAngle(ArmConstants.DOWN_ANGLE);
    }

    public Command raiseIntake() {
        return this.arm.setAngle(ArmConstants.UP_ANGLE);
    }
}
