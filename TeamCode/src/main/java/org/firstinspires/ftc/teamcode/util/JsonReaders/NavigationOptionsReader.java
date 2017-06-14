/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util.JsonReaders;

import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


public class NavigationOptionsReader extends JsonReader {
    String navOptStr;
    public JSONObject navOptObj;
    public JSONObject lfObj=null;
    public JSONObject imuObj=null;
    public JSONObject rangeObj=null;
    public JSONObject encoderVarsObj=null;
    public JSONObject coordinateSysObj=null;

    public NavigationOptionsReader(String filePath, String navOptStr) {
        super(filePath);
        try {
            String key = JsonReader.getRealKeyIgnoreCase(jsonRoot, navOptStr);
            this.navOptObj = jsonRoot.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "LineFollow");
            if (key != null) {
                lfObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "IMU");
            if (key != null) {
                imuObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "RangeSensor");
            if (key != null) {
                rangeObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "DriveSysEncoderVariables");
            if (key != null) {
                encoderVarsObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "CoordinateSys");
            if (key != null){
                coordinateSysObj = navOptObj.getJSONObject(key);
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public boolean lineFollowerExists() {
        return (this.lfObj != null);
    }

    public boolean imuExists() {
        return (this.imuObj != null);
    }

    public boolean rangeSensorExists() { return (this.rangeObj != null); }

    public boolean encoderVarsExist() { return (this.encoderVarsObj != null); }

    public boolean coordinateSysExists(){return (this.coordinateSysObj != null);}

    public String getLightSensorName() {
        String lightSensorName = null;
        String key=null, value1=null, value2=null;
        JSONObject lightSensorObj;
        try {
            key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensor");
            lightSensorObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "name");
            lightSensorName = lightSensorObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
            DbgLog.msg("Single Light sensor not found; trying 2 light sensors");
            try {
                key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensorFront");
                lightSensorObj = lfObj.getJSONObject(key);
                key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "name");
                value1 = lightSensorObj.getString(key);
                key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensorBack");
                lightSensorObj = lfObj.getJSONObject(key);
                key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "name");
                value2 = lightSensorObj.getString(key);
                lightSensorName = String.format("%s,%s", value1, value2);
            } catch (JSONException e1) {
                DbgLog.msg("Neight single nor 2 light sensors found");
                e1.printStackTrace();
            }
        }
        return (lightSensorName);
    }

    public String getLightSensorType() {
        String sensorType = null;
        JSONObject lightSensorObj;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensor");
            lightSensorObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "type");
            sensorType = lightSensorObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (sensorType);
    }

    public double getLFvariableDouble(String variableName) {
        double value = 0.0;
        JSONObject lfVarObj;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "lineFollowVariables");
            lfVarObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfVarObj, variableName);
            value = lfVarObj.getDouble(key);
            DbgLog.msg("ftc9773: getLFvariableDouble(): key = %s, value=%f", key, value);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }

    public String getIMUType() {
        String imuType = null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "type");
            imuType = imuObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (imuType);
    }

    public String getIMUDIMname() {
        String imuName = null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "DIMname");
            imuName = imuObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (imuName);
    }

    public int getIMUportNum() {
        int imuPortNum = 0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "portnum");
            imuPortNum = imuObj.getInt(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (imuPortNum);
    }

    public double getIMUVariableDouble(String variableName) {
        double value=0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, variableName);
            value = imuObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }

    public double getRangeSensorRunningAvgWeight() {
        double value = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(rangeObj, "runningAvgWeight");
            value = rangeObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);

    }

    public double getDoubleDriveSysEncVar(String varName) {
        double maxSpeed = 1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, varName);
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getTurningMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "TurningMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getStraightLineMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "StraightLineMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getLineFollowMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "LineFollowMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getStartingPositionX(){
        double startingX=0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(coordinateSysObj, "startingX");
            startingX = coordinateSysObj.getDouble(key);
        } catch (JSONException e){
            e.printStackTrace();
        }
        return startingX;
    }

    public double getStartingPositionY(){
        double startingY=0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(coordinateSysObj, "startingY");
            startingY = coordinateSysObj.getDouble(key);
        } catch (JSONException e){
            e.printStackTrace();
        }
        return startingY;
    }

    public double getStartingPositionAngle(){
        double startingAngle=0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(coordinateSysObj, "startingAngle");
            startingAngle = coordinateSysObj.getDouble(key);
        } catch (JSONException e){
            e.printStackTrace();
        }
        return startingAngle;
    }
}
