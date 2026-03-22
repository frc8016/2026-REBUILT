// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

/** Our implementation of 6328's TunableNumber class. */
public class TunableNumber implements AutoCloseable {
    private static ArrayList<TunableNumber> tunableNumbers = new ArrayList<TunableNumber>();

    private String smartDashboardName;
    private double lastCheckedValue;
    private double value;

    public TunableNumber(String smartDashboardName, double defaultValue) {
        this.smartDashboardName = smartDashboardName;
        lastCheckedValue = defaultValue;
        value = defaultValue;
        SmartDashboard.putNumber(smartDashboardName, defaultValue);
        tunableNumbers.add(this);
    }

    /**
     * This is run automatically. Prevents 'updateAll()' from trying to update a deleted
     * TunableNumber.
     */
    @Override
    public void close() {
        tunableNumbers.remove(this);
    }

    /** Update the value of all TunableNumbers. */
    public static void updateAll() {
        // it's not particularly necessary to do this periodically, instead
        // update the SmartDashboard value and check whether it has changed
        // whenever 'hasChanged' is called (just make sure to check the robot)
        // is in an acceptable mode first (i.e. isDisabled())
        for (TunableNumber tunableNumber : tunableNumbers) {
            tunableNumber.update();
        }
    }

    public boolean hasChanged() {
        final boolean hasChanged = lastCheckedValue != value;
        lastCheckedValue = value;
        return hasChanged;
    }

    public double get() {
        return value;
    }

    public void update() {
        value = SmartDashboard.getNumber(smartDashboardName, Double.NaN);
    }
}
