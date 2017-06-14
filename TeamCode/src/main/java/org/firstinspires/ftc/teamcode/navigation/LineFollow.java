/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by Luke on 10/15/2016.
 */

public class LineFollow{

    OpticalDistanceSensor lightSensorFront=null, lightSensorBack=null;
    double white, black, mid;
    double lowSpeed, highSpeed;
    double odsOffset;
    double basePower, Kp;
    DriveSystem driveSystem;
    long stopTimeStamp=0;
    long timoutNanoSec=0;
    FTCRobot robot;

    public LineFollow(FTCRobot robot, String lightSensorName, double lowSpeed,
                      double highSpeed, double lineFollowTimeOut,
                      double white, double black) {
        this.robot = robot;
        this.driveSystem = robot.driveSystem;
        this.lowSpeed = lowSpeed;
        this.highSpeed = highSpeed;
        this.basePower = (lowSpeed+highSpeed)/2;
        this.white = white;
        this.black = black;
        this.mid = (white + black) / 2;
        //this.Kp = (highSpeed-this.basePower) / (white - this.mid);
        this.Kp = 0.5;
        this.odsOffset = robot.distanceLeft / (robot.distanceLeft + robot.distanceRight);
        this.timoutNanoSec = (long) (lineFollowTimeOut * 1000000000L);
        if (lightSensorName.contains(",")) {
            // 2 ODS sensors
            String[] sensorNames = lightSensorName.split(",");
            this.lightSensorFront = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(sensorNames[0]);
            this.lightSensorBack = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(sensorNames[1]);
        } else {
            this.lightSensorBack = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        }
        DbgLog.msg("ftc9773: sensorName=%s, lowSpeed=%f, highSpeed=%f, timeoutNanoSec=%d",
                lightSensorName, lowSpeed, highSpeed, this.timoutNanoSec);
        DbgLog.msg("ftc9773: Kp = %f, odsOffset=%f", this.Kp, this.odsOffset);
//        this.white = -1;
//        this.black = -1;
    }

    public void turnUntilWhiteLine(boolean spinClockwise) {
        double leftInitialPower=0.0, rightInitialPower=0.0;
        if(spinClockwise){
            leftInitialPower = 0.3;
            rightInitialPower = -leftInitialPower;
        }
        else{
            leftInitialPower = -0.3;
            rightInitialPower = -leftInitialPower;
        }
        while ((lightSensorBack.getLightDetected()<this.mid) && robot.curOpMode.opModeIsActive()) {
            driveSystem.turnOrSpin(leftInitialPower,rightInitialPower);
//            if (lightSensor.getLightDetected()<this.mid)
//                break;
        }
        driveSystem.stop();

    }

    public void printMinMaxLightDetected(double milliseconds) {
        double minLight_f = 1.0, maxLight_f=0.0, curLight_f;
        double minLight_b = 1.0, maxLight_b=0.0, curLight_b;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        robot.driveSystem.drive(0.4f, 0);
        while ((timer.milliseconds() < milliseconds) && robot.curOpMode.opModeIsActive()) {
            curLight_f = getLightDetectedFront();
            curLight_b = getLightDetectedBack();
            if (minLight_f > curLight_f) minLight_f = curLight_f;
            if (minLight_b > curLight_b) minLight_b = curLight_b;
            if (maxLight_f < curLight_f) maxLight_f = curLight_f;
            if (maxLight_b < curLight_b) maxLight_b = curLight_b;
            DbgLog.msg("ftc9773: minLight_f=%f, maxLight_f=%f, curLight_f=%f",
                    minLight_f, maxLight_f, curLight_f);
            DbgLog.msg("ftc9773: minLight_b=%f, maxLight_b=%f, curLight_b=%f",
                    minLight_b, maxLight_b, curLight_b);
        }
        driveSystem.stop();
        robot.curOpMode.telemetry.addData("Light Detected (frontODS):", "minLight_f=%f, maxLight_f=%f", minLight_f, maxLight_f);
        robot.curOpMode.telemetry.addData("Light Detected (backODS):", "minLight_b=%f, maxLight_b=%f", minLight_b, maxLight_b);
        robot.curOpMode.telemetry.update();
    }

    public double getLightDetectedFront() {
        if (lightSensorFront != null) {
            return (lightSensorFront.getLightDetected());
        } else {
            return (0.0);
        }
    }

    public double getLightDetectedBack() {
        if (lightSensorBack != null) {
            return (lightSensorBack.getLightDetected());
        } else {
            return (0.0);
        }
    }

    public boolean FrontODSonWhiteLine() {
        return (lightSensorFront.getLightDetected() >= this.mid);
    }
    public boolean BackODSonWhiteLine() {
        return (lightSensorBack.getLightDetected() >= this.mid);
    }
}
