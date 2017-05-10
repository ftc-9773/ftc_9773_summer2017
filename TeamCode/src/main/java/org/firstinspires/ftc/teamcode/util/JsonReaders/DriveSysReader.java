/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by pranavb on 10/15/16.
 */

public class DriveSysReader extends JsonReader {
    JSONObject driveSysObj;
    String driveSysName;

    public DriveSysReader(String filePath, String driveSysName)
    {
        super(filePath);
        try {
            driveSysName = JsonReader.getRealKeyIgnoreCase(jsonRoot, driveSysName);
            driveSysObj = jsonRoot.getJSONObject(driveSysName);
            this.driveSysName = driveSysName;
        }catch (JSONException e){
            e.printStackTrace();
        }
    }

    public String getMotorType(String motorName) {
        String motorType = null;
        JSONObject obj;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(driveSysObj, "Motors");
            obj = driveSysObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(obj, motorName);
            motorType = obj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (motorType);
    }

    public String getWheelType(){
        String wheelType = null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(driveSysObj, "WheelType");
            wheelType = driveSysObj.getString(key);
        } catch (JSONException e){
            e.printStackTrace();
        }

        return wheelType;
    }

    public String getDriveSysName() {
        return driveSysName;
    }

    public int getMaxMotorSpeed(String autoOrTeleop) {
        int maxMotorSpeed = 0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(driveSysObj, autoOrTeleop);
            maxMotorSpeed = driveSysObj.getInt(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxMotorSpeed);
    }
}
