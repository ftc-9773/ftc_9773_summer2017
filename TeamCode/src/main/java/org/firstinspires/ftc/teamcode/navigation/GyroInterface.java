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
    boolean isGyroWorking();
    void testAndSetGyroStatus();
    double getUpdateCount();
    double getAngleTolerance();
    //ToDo:  Create a close() method

    void goStraightPID(boolean driveBackwards, double degrees, float speed);
}
