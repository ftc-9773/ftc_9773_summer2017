/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by ftcrobocracy on 12/26/16.
 */

public class EncoderNavigation {
    private double currentYaw = 0.0;
    private FTCRobot robot;
    private DriveSystem driveSys;
    private LinearOpMode curOpMode;
    private Navigation navigation;

    public EncoderNavigation(FTCRobot robot, DriveSystem driveSys, LinearOpMode curOpMode,
                             Navigation navigation) {
        this.robot = robot;
        this.driveSys = driveSys;
        this.curOpMode = curOpMode;
        this.navigation = navigation;
    }

    public void updateCurrentYaw(double degreesTurned){
        currentYaw += degreesTurned;
        if (currentYaw < 0){
            currentYaw += 360;
        }
        else if (currentYaw > 360){
            currentYaw -= 360;
        }
    }

    /**
     * Sets the encoderNavigation's private variable currentYaw to the given value.
     * This method is called when navx is working well and we want the curYaw to have the
     * same value as navxMicro.getYaw().
     * @param yawValue
     */
    public void setCurrentYaw(double yawValue) {
        this.currentYaw = yawValue;
    }

    public void setRobotOrientation (double targetYaw, double speed, NavigationChecks navExc) {
        double degrees = navigation.getDegreesToTurn(currentYaw, targetYaw);

        DbgLog.msg("ftc9773: degrees: %f, currYaw: %f, targetYaw: %f", degrees, this.getCurrentYaw(), targetYaw);

        driveSys.turnDegrees(degrees, (float)speed, navExc);
    }

    public void shiftRobot(double distance, double moveDistance, boolean isForward, double speed,
                           NavigationChecks navigationChecks, boolean returnToSamePos){
        double driveDistance = Math.sqrt(Math.pow(moveDistance, 2) + Math.pow(distance, 2));
        double angle = 90 - Math.toDegrees(Math.asin(moveDistance/driveDistance));

        if (isForward){
            if (distance < 0) {
                angle *= -1;
            }
            driveSys.turnDegrees(angle, 0.5f, navigationChecks);
            driveSys.driveToDistance((float)speed, driveDistance);
            navigationChecks.reset();
            driveSys.turnDegrees(-angle, 0.5f, navigationChecks);
            driveSys.driveToDistance((float)speed, -moveDistance);
        }
        else{
            if (distance > 0){
                angle *= -1;
            }
            driveSys.turnDegrees(angle, 0.5f, navigationChecks);
            driveSys.driveToDistance((float)speed, -driveDistance);
            navigationChecks.reset();
            driveSys.turnDegrees(-angle, 0.5f, navigationChecks);
            driveSys.driveToDistance((float)speed, moveDistance);
        }
    }


    public double getCurrentYaw() {
        return currentYaw;
    }
}
