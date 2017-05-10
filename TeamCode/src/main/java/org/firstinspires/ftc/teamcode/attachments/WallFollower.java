/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by ftcrobocracy on 3/11/17.
 */

public class WallFollower implements Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    private CRServo wallServoCR =null;
    private double timeToUnfold = 500; // milli seconds


    public WallFollower(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        String key=null;
        JSONObject motorsObj=null, wallServoObj=null;
        JSONObject wallJsonObj=null;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "WallFollower");
            wallJsonObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(wallJsonObj, "motors");
            motorsObj = wallJsonObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }

        try{
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "wallFollowCR");
            wallServoObj = motorsObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }

        if (wallServoObj != null) {
            String servoType=null;
            try {
                key = JsonReader.getRealKeyIgnoreCase(wallServoObj, "motorType");
                servoType = wallServoObj.getString(key);
                DbgLog.msg("ftc9773: wall follow motorType: %s", servoType);
            } catch (JSONException e) {
                e.printStackTrace();
            }

            try {
                if (servoType.equalsIgnoreCase("CRservo")) {
                    wallServoCR = curOpMode.hardwareMap.crservo.get("wallServo");
                    if (wallServoObj.getBoolean("needReverse")) {
                        DbgLog.msg("ftc9773: Reversing the wall servo");
                        wallServoCR.setDirection(CRServo.Direction.REVERSE);
                    }
                }
            } catch (JSONException e) {
                e.printStackTrace();
            } catch (IllegalArgumentException e) {
                DbgLog.msg("ftc9773:  IllegalArgumentException has occurred");
                e.printStackTrace();
            }

            try {
                key = JsonReader.getRealKeyIgnoreCase(wallServoObj, "timeToUnfold");
                timeToUnfold = wallServoObj.getDouble(key);
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
    }

    private void unfold() {
        wallServoCR.setPower(1.0);
    }

    private void fold() {
        wallServoCR.setPower(-1.0);
    }

    public void idle() {
        wallServoCR.setPower(0.0);
    }

    public void activateWallFollwer(double millis) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while ((timer.milliseconds() < millis) && curOpMode.opModeIsActive()) {
            unfold();
        }
    }

    public void deactivateWallFollower(double millis) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while ((timer.milliseconds() < millis) && curOpMode.opModeIsActive()) {
            fold();
        }
    }

}
