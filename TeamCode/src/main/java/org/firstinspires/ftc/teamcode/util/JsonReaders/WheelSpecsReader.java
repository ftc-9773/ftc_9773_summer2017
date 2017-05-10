/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;


import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class WheelSpecsReader extends JsonReader {
    JSONObject wheelSpecObj;
    String wheelType;
    public WheelSpecsReader(String filePath, String wheelType) {

        super(filePath);
        try {
            wheelType = JsonReader.getRealKeyIgnoreCase(jsonRoot, wheelType);
            wheelSpecObj =  jsonRoot.getJSONObject(wheelType);
            this.wheelType = wheelType;
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public double getDiameter() {
        double diameter = 0.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(wheelSpecObj, "diameter");
            diameter = wheelSpecObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (diameter);
    }

    public double getFrictionCoeff() {
        double frictionCoeff = 1.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(wheelSpecObj, "friction_coeff");
            frictionCoeff = wheelSpecObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (frictionCoeff);
    }
}
