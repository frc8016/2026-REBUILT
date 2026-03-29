package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeedConstants;

public class Feed extends SubsystemBase {

    private final SparkMax feed =
            new SparkMax(1, MotorType.kBrushless); // TODO: motor id must be changed
    private final SparkMaxConfig feedConfig = new SparkMaxConfig();

    public Feed() {
        feedConfig.inverted(true);

        feed.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command run() {
        return new StartEndCommand(
                () -> this.feed.set(FeedConstants.FEED_SPEED), () -> this.feed.set(0), this);
    }

    public Command reverse() {
        return new StartEndCommand(
                () -> this.feed.set(-FeedConstants.FEED_SPEED), () -> this.feed.set(0), this);
    }
}
