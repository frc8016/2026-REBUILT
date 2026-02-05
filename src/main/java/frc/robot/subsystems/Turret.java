package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TurretConstants;

// import frc.robot.LimelightHelpers;

public class Turret {
    private final SparkMax m_turretmotor = new SparkMax(15, MotorType.kBrushless);
    private final SparkMaxConfig m_turretmotorconfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_turretmotorClosedLoopController =
            m_turretmotor.getClosedLoopController();

    // program motor
    // private final LimelightHelpers LimelightHelpers = new LimelightHelpers();
    public Turret() {
        m_turretmotorconfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(TurretConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                .i(TurretConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                .d(TurretConstants.D_VALUE, ClosedLoopSlot.kSlot0)
                .outputRange(
                        TurretConstants.OUTPUTRANGE_MIN_VALUE,
                        TurretConstants.OUTPUTRANGE_MAX_VALUE);
    }

    public Command aimShooter() {
        return Commands.runOnce(() -> {});
    }

    // private Command Turn
    private void trunMotors() {
        while (true) {}
    }

    // set a while loop outside of command that checks up on lime data( treshhold are we aimed or
    // not
    // ) - with that data aim motor

}
