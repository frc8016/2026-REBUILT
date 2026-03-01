package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Turret extends SubsystemBase {
    private final SparkMax m_turretmotor = new SparkMax(3, MotorType.kBrushless);
    private final SparkMaxConfig m_turretmotorconfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_turretmotorClosedLoopController =
            m_turretmotor.getClosedLoopController();
    private final LimelightManager limelightManager;
    SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            TurretConstants.P_VALUE,
                            TurretConstants.I_VALUE,
                            TurretConstants.D_VALUE,
                            DegreesPerSecond.of(180),
                            DegreesPerSecondPerSecond.of(90))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withMotorInverted(false)
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimit(Degrees.of(-180), Degrees.of(180));

    public Turret(TargetManager targetManager) {
        this.targetManager = targetManager;
        m_turretmotorconfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(TurretConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                .i(TurretConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                .d(TurretConstants.D_VALUE, ClosedLoopSlot.kSlot0)
                .outputRange(
                        TurretConstants.OUTPUTRANGE_MIN_VALUE,
                        TurretConstants.OUTPUTRANGE_MAX_VALUE);
        m_turretmotor.configure(
                m_turretmotorconfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public void runTurret() {
        double tx = targetManager.TX().get().doubleValue();
        if (!MathUtil.isNear(tx, 0, Constants.TurretConstants.TX_TOLERANCE)) {
            m_turretmotorClosedLoopController.setSetpoint(tx, ControlType.kVelocity);

        } else {
            m_turretmotorClosedLoopController.setSetpoint(0, ControlType.kVelocity);
        }
    }

    public Command Autoaim() {
        return this.runOnce(() -> this.runTurret());
    }
}
