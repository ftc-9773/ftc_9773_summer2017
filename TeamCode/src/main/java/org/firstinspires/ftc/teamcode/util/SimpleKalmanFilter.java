/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ftcrobocracy on 2/15/17.
 */

public class SimpleKalmanFilter {
    double process_error, measurement_error;
    double change_per_10ms; // change in the value per 10 milli seconds
    // The robot turns/spins at the rate of 1 degree per 10 milli seconds
    ElapsedTime timer;
    double prev_value;
    public SimpleKalmanFilter(double starting_value, double process_error, double measurement_error,
                              double change_per_10ms) {
        this.process_error = process_error;
        this.measurement_error = measurement_error;
        this.change_per_10ms = change_per_10ms;
        this.prev_value = starting_value;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
    }

    public void reInitialize(double starting_value, double process_error,
                             double measurement_error, double change_per_10ms) {
        this.process_error = process_error;
        this.measurement_error = measurement_error;
        this.change_per_10ms = change_per_10ms;
        this.prev_value = starting_value;
        timer.reset();
    }

    public double estimateNextValue(double measuredValue) {
        double modeledValue = prev_value + (change_per_10ms * timer.milliseconds() / 10);
        timer.reset();
        double gain = process_error / (process_error + measurement_error);
        double estimatedValue = modeledValue + gain * (measuredValue - modeledValue);
        process_error = (1 - gain) * process_error;
        prev_value = estimatedValue;
        return (estimatedValue);
    }
}
