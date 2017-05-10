/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;

import com.google.gson.JsonObject;
import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

/**
 * Created by pranavb on 10/15/16.
 */

public class RobotConfigReader extends JsonReader {
    String robotName;
    JSONObject robotObj=null;
    public RobotConfigReader(String filePath, String robotName)
    {
        super(filePath);
        this.robotName = robotName;
        try {
            this.robotObj = jsonRoot.getJSONObject(robotName);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public String getDriveSysName(){
        String name = null;
        try{
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "driveSystem");
            name = robotObj.getString(key);
        }catch (JSONException e){
            e.printStackTrace();
        }
        return name;
    }

    public String getNavigationOption(String autoOrTeleop) {
        // ToDo
        String navigationOption = null;
        String key=null;
        try{
            if (autoOrTeleop.equalsIgnoreCase("Autonomous")) {
                key = JsonReader.getRealKeyIgnoreCase(robotObj, "autonomous_navigation");
            } else if (autoOrTeleop.equalsIgnoreCase("Teleop")) {
                key = JsonReader.getRealKeyIgnoreCase(robotObj, "teleop_navigation");
            }
            navigationOption = robotObj.getString(key);
        }catch (JSONException e){
            e.printStackTrace();
            DbgLog.error("ftc9773: navigation key not found for the robot named %s!", robotName);
        }
        return (navigationOption);
    }

    public String getString(String key) {
        String value=null;
        try {
            key = JsonReader.getRealKeyIgnoreCase(robotObj, key);
            value = robotObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
            DbgLog.error("ftc9773: key %s not found for the robot named %s!", key, robotName);
        }
        return (value);
    }

    public String[] getAttachments(String autoOrTeleop) {
        int len = 0;
        String[] attachmentsArr = null;
        JSONArray attachs=null;
        try {
            if (autoOrTeleop.equalsIgnoreCase("Autonomous")) {
                attachs = robotObj.getJSONArray("autonomous_attachments");
            } else if (autoOrTeleop.equalsIgnoreCase("Teleop")) {
                attachs = robotObj.getJSONArray("teleop_attachments");
            }
            len = attachs.length();
            DbgLog.msg("ftc9773: Length of attachs array = %d", attachs.length());
            attachmentsArr = new String[len];
            for (int i = 0; i < len; i++) {
                attachmentsArr[i] = attachs.getString(i);
            }
        } catch (JSONException e) {
            e.printStackTrace();
            DbgLog.error("ftc9773: Problem finding one or more attachments for the robot named %s",
                    robotName);
        }
        return (attachmentsArr);
    }

    // This method is deprecated as we are not doing line follow anymore
    @Deprecated
    public double getDistanceLeft() {
        double value = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "distanceBetweenLeftAndODS");
            value = robotObj.getDouble(key);
            DbgLog.msg("ftc9773: getDistanceLeft(): key = %s, value=%f", key, value);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }

    // This method is deprecated as we are not doing line follow anymore
    @Deprecated
    public double getDistanceRight() {
        double value = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "distanceBetweenRightAndODS");
            value = robotObj.getDouble(key);
            DbgLog.msg("ftc9773: getDistanceRight(): key = %s, value=%f", key, value);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }

    public double getDistanceBetweenWheels() {
        double value = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "distanceBetweenWheels");
            value = robotObj.getDouble(key);
            DbgLog.msg("ftc9773: getDistanceBetweenWheels(): key = %s, value=%f", key, value);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }
}
