/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;


import org.json.JSONException;
import org.json.JSONObject;


public class ServoSpecsReader extends JsonReader {
    JSONObject servoObj;
    String servoModel;

    public ServoSpecsReader(String filePath, String servoModel) {
        super(filePath);
        try {
            servoModel = JsonReader.getRealKeyIgnoreCase(jsonRoot, servoModel);
            servoObj = jsonRoot.getJSONObject(servoModel);
            this.servoModel = servoModel;
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public double getRotation() {
        double rotation = 0.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(servoObj, "rotation");
            rotation = servoObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (rotation);
    }
}
