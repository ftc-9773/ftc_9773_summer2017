/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;


import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;



public class MotorSpecsReader extends JsonReader {
    JSONObject motorObj;
    String motorType;

    public MotorSpecsReader(String filePath, String motorType) {
        super(filePath);
        try {
            motorType = JsonReader.getRealKeyIgnoreCase(jsonRoot, motorType);
            motorObj = jsonRoot.getJSONObject(motorType);
            this.motorType = motorType;
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public double getMaxSpeed() {
        double maxSpeed = 0.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(motorObj, "maxSpeed");
            maxSpeed = motorObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public int getCPR() {
        int CPR = 0; // Counts Per Revolution

        try {
            String key = JsonReader.getRealKeyIgnoreCase(motorObj, "CPR");
            CPR = motorObj.getInt(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (CPR);

    }
}
