/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.navigation;

/**
 * Created by ftcrobocracy on 2/20/17.
 */

public interface GyroInterface {
    void initAfterStart();
    Navigation.GyroType getGyroType();

    double getYaw();
    double getPitch();
    double getRoll();
    // The initial angle is 0; the angle changes from 0 to 180  and then
    // from -180 to 0 as the robot spins counter-clockwise (cc)
    double getYaw_cc0_180scale();
    boolean isGyroWorking();
    void testAndSetGyroStatus();
    double getUpdateCount();
    double getAngleTolerance();
    void close();

    void goStraightPID(boolean driveBackwards, double degrees, float speed);
}
