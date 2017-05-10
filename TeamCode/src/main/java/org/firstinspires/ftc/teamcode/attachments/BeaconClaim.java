/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class BeaconClaim implements Attachment {
    private FTCRobot robot;
    private LinearOpMode curOpMode;
    private CRServo buttonServoCR =null;
    private Servo buttonServoLinear=null;
    private Servo buttonServoStandard=null;
//    private Servo buttonWheelServo =null;
    private ModernRoboticsI2cColorSensor colorSensor1=null;
    public enum BeaconColor {RED, BLUE, NONE}
    public enum IndexType {REDMAX_FIRST, BLUEMAX_FIRST, REDMAX_LAST, BLUEMAX_LAST, CURRENT}
    public BeaconColor beaconColor;

    public class BeaconScanData {
        int[] redValues;
        int[] blueValues;
        long[] elapsedMillis; // elapsed time in milliseconds from the beginning
        double[] distFromStart; // distance in inches from the beginning
        int firstRedMaxIndex, firstBlueMaxIndex, lastRedMaxIndex, lastBlueMaxIndex;
        int lastValidIndex;
        boolean redBeaconFound, blueBeaconFound;
        String allianceColor;
        BeaconColor myAllianceColor, otherAllianceColor;
        int numEntries;

        public BeaconScanData(int numEntries, String allianceColor) {
            this.numEntries = numEntries;
            this.allianceColor = allianceColor;
            myAllianceColor = (allianceColor.equalsIgnoreCase("red")) ? BeaconColor.RED : BeaconColor.BLUE;
            otherAllianceColor = (myAllianceColor == BeaconColor.RED) ? BeaconColor.BLUE : BeaconColor.RED;
            redValues = new int[numEntries];
            blueValues = new int[numEntries];
            elapsedMillis = new long[numEntries];
            distFromStart = new double[numEntries];
            firstRedMaxIndex = firstBlueMaxIndex = lastRedMaxIndex = lastBlueMaxIndex = -1;
            lastValidIndex = -1;
            for (int i=0; i<numEntries; i++) {
                redValues[i] = blueValues[i] = 0;
                elapsedMillis[i] = 0;
                distFromStart[i] = 0.0;
            }
            redBeaconFound = blueBeaconFound = false;
        }

        public void addAnEntry(int redValue, int blueValue, long millis, double distTravelled) {
            lastValidIndex++;
            if (lastValidIndex <0 || lastValidIndex>numEntries) {
                curOpMode.telemetry.addData("BeaconClaimScanData:", "Error! Too many values");
                curOpMode.telemetry.update();
                DbgLog.error("ftc9773: BeaconClaimScanData: Error! array size exceeded!");
            }
            return;
        }

        public double distanceToMyAllianceColor() {
            return (0);
        }

        public double distanceToOtherAllianceColor() {
            return (0);
        }
    }

    private double curLength;
    double buttonServoSpeed; // units: cm per second

    double strokeLength; // units: cm
    public enum BeaconClaimOperation {EXTEND, RETRACT, NONE}
    private BeaconClaimOperation lastOp;
    private ElapsedTime lastOpTimer;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj, String beaconClaimStrToRead) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        String key=null;
        JSONObject beaconJsonObj=null;
        JSONObject motorsObj=null, buttonServoObj=null, colorServoObj=null;
//        JSONObject buttonWheelServoObj=null;
        JSONObject sensorsObj = null,colorSensor1Obj=null;

        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, beaconClaimStrToRead);
            beaconJsonObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(beaconJsonObj, "motors");
            motorsObj = beaconJsonObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(beaconJsonObj, "sensors");
            sensorsObj = beaconJsonObj.getJSONObject(key);
        } catch (JSONException e) {
            DbgLog.error("ft9773: Caanot read json object %s", key);
            e.printStackTrace();
        }

        try {
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServoCR");
            if (key == null)
                key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServoLinear");
            if (key == null)
                key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServoStandard");
            buttonServoObj = motorsObj.getJSONObject(key);
//            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonWheelServo");
//            buttonWheelServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            colorSensor1Obj = sensorsObj.getJSONObject(key);

        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        if (buttonServoObj != null) {
            String servoType=null;
            try {
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "motorType");
                servoType = buttonServoObj.getString(key);
                DbgLog.msg("ftc9773: motorType: %s", servoType);
            } catch (JSONException e) {
                e.printStackTrace();
            }

            try {
                if (servoType.equalsIgnoreCase("CRservo")) {
                    buttonServoCR = curOpMode.hardwareMap.crservo.get("buttonServo");
                    if (buttonServoObj.getBoolean("needReverse")) {
                        DbgLog.msg("ftc9773: Reversing the button servo");
                        buttonServoCR.setDirection(CRServo.Direction.REVERSE);
                    }
                } else if (servoType.equalsIgnoreCase("LinearServo")) {
                    buttonServoLinear = curOpMode.hardwareMap.servo.get("buttonServo");
                    if (buttonServoObj.getBoolean("needReverse")) {
                        DbgLog.msg("ftc9773: Reversing the button servo");
                        buttonServoLinear.setDirection(Servo.Direction.REVERSE);
                    }
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMin");
                    double scaleMin = buttonServoObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMax");
                    double scaleMax = buttonServoObj.getDouble(key);
                    buttonServoLinear.scaleRange(scaleMin, scaleMax);
                    buttonServoLinear.setPosition(0.0);
                } else if (servoType.equalsIgnoreCase("StandardServo")){
                    buttonServoStandard = curOpMode.hardwareMap.servo.get("buttonServo");
                    if (buttonServoObj.getBoolean("needReverse")){
                        DbgLog.msg("ftc9773: Rervsing the button servo");
                        buttonServoStandard.setDirection(Servo.Direction.REVERSE);
                    }
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMin");
                    double scaleMin = buttonServoObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMax");
                    double scaleMax = buttonServoObj.getDouble(key);
                    buttonServoStandard.scaleRange(scaleMin, scaleMax);
                    buttonServoStandard.setPosition(0.0);
                }
                // speed and strokeLength parameters are common to both types of servos
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "speed");
                buttonServoSpeed = buttonServoObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "strokeLength");
                strokeLength = buttonServoObj.getDouble(key);
            } catch (IllegalArgumentException e) {
                DbgLog.msg("ftc9773:  IllegalArgumentException has occurred");
                e.printStackTrace();
            }catch (JSONException e) {
                // Based on tests, it takes 200 milliseconds per 1 cm of extension
                DbgLog.msg("ftc9773: JSON exception occurred.  key =%s", key);
                buttonServoSpeed = 5.0; // default value = 5 cm per second
                strokeLength = 15.0; // Just in case the JSON reader failed, set the default in 15 cm
                e.printStackTrace();
            }

            lastOp = BeaconClaimOperation.NONE;
            lastOpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            lastOpTimer.reset();
            DbgLog.msg("ftc9773: buttonServoSpeed=%f, strokeLength=%f",
                    buttonServoSpeed, strokeLength);
        }
        if (colorSensor1Obj != null) {
            DbgLog.msg("ftc9773: color sensor not null");
            colorSensor1 = curOpMode.hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor1");
            // "blink" the color sensor so that we can visually see that it is working
            colorSensor1.enableLed(false);
            curOpMode.sleep(100);
            colorSensor1.enableLed(true);
            curOpMode.sleep(100);
            colorSensor1.enableLed(false);
        }
//        if (buttonWheelServoObj != null){
//            DbgLog.msg("ftc9773: buttonWheelServo not null");
//            buttonWheelServo = curOpMode.hardwareMap.servo.get("buttonWheelServo");
//            try {
//                key = JsonReader.getRealKeyIgnoreCase(buttonWheelServoObj, "scaleRangeMin");
//                double scaleRangeMin = buttonWheelServoObj.getDouble(key);
//                key = JsonReader.getRealKeyIgnoreCase(buttonWheelServoObj, "scaleRangeMax");
//                double scaleRangeMax = buttonWheelServoObj.getDouble(key);
//                buttonWheelServo.scaleRange(scaleRangeMin, scaleRangeMax);
//                if (buttonWheelServoObj.getBoolean("needReverse")){
//                    DbgLog.msg("ftc9773: Reversing buttonWheelServo");
//                    buttonWheelServo.setDirection(Servo.Direction.REVERSE);
//                }
//                buttonWheelServo.setPosition(0);
//            } catch (JSONException e){
//                e.printStackTrace();
//            }
//        }

        beaconColor = BeaconColor.NONE;
        curLength = 0.0;
    }

//    public void lowerButtonWheel(){
//        buttonWheelServo.setPosition(1);
//    }
//    public void raiseButtonWheel(){
//        buttonWheelServo.setPosition(0);
//    }

    public double getCurLength() {
        return curLength;
    }

    private void updateBeaconServoLength(BeaconClaimOperation op) {
        if (lastOp == BeaconClaimOperation.NONE) {
            lastOpTimer.reset();
        } else if (lastOp == BeaconClaimOperation.EXTEND) {
            curLength += lastOpTimer.milliseconds() * buttonServoSpeed / 1000;
            curLength =  (curLength > strokeLength) ? strokeLength : curLength;
        } else {
            curLength -= lastOpTimer.milliseconds() * buttonServoSpeed / 1000;
            curLength = (curLength < 0) ? 0 : curLength;
        }
        lastOpTimer.reset();
        lastOp = op;
    }
    public void pushBeacon(){
        if (buttonServoCR != null) {
//            updateBeaconServoLength(BeaconClaimOperation.EXTEND);

            buttonServoCR.setPower(-1.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.EXTEND;
            curLength = Range.clip(curLength+1.0, 0, strokeLength);
            buttonServoLinear.setPosition(curLength / strokeLength);
        } else if (buttonServoStandard != null){
            lastOp = BeaconClaimOperation.EXTEND;
            curLength =  Range.clip(curLength+1.0, 0, strokeLength);
            buttonServoStandard.setPosition(1.0);
        }
    }
    public void retractBeacon(){
        if (buttonServoCR != null) {
//            updateBeaconServoLength(BeaconClaimOperation.RETRACT);
            buttonServoCR.setPower(1.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.RETRACT;
            curLength = Range.clip(curLength-1.0, 0, strokeLength);
            buttonServoLinear.setPosition(curLength / strokeLength);
        } else if (buttonServoStandard != null){
            lastOp = BeaconClaimOperation.RETRACT;
            curLength = Range.clip(curLength-1.0, 0, strokeLength);
            buttonServoStandard.setPosition(0.0);
        }
    }
    public void idleBeacon(){
        if (buttonServoCR != null) {
//            updateBeaconServoLength(BeaconClaimOperation.NONE);
            buttonServoCR.setPower(0.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.NONE;
        } else if (buttonServoStandard != null){
            lastOp = BeaconClaimOperation.NONE;
        }
    }

    public void activateButtonServo(double timeToExtend, double lengthToExtend) {
        if (buttonServoCR != null) {
            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            elapsedTime.reset();
            while (elapsedTime.milliseconds() < timeToExtend && curOpMode.opModeIsActive()) {
                pushBeacon();
            }
//        curOpMode.sleep(500);
            idleBeacon();
            curLength += (timeToExtend * buttonServoSpeed);
            curLength = (curLength > strokeLength) ? strokeLength : curLength;
        } else if (buttonServoLinear != null) {
            double servoPosition = Range.clip(lengthToExtend, 0, strokeLength) / strokeLength;
            servoPosition = Range.clip(buttonServoLinear.getPosition() + servoPosition, 0, 1);
            DbgLog.msg("ftc9773: activateButtonServo: cur position = %f, new position=%f",
                    buttonServoLinear.getPosition(), servoPosition);
            buttonServoLinear.setPosition(servoPosition);
            curOpMode.sleep((long)timeToExtend);
        }
    }

    public void deactivateButtonServo(double timeToRetract, double lengthToRetract) {
        if (buttonServoCR != null) {
            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            elapsedTime.reset();
            while ((elapsedTime.milliseconds() < timeToRetract) && curOpMode.opModeIsActive()) {
                retractBeacon();
            }
//        curOpMode.sleep(500);
            idleBeacon();
            curLength -= (timeToRetract * buttonServoSpeed);
            curLength = (curLength < 0) ? 0 : curLength;
        } else if (buttonServoLinear != null) {
            double servoPosition = Range.clip(lengthToRetract, 0, strokeLength) / strokeLength;
            servoPosition = Range.clip(buttonServoLinear.getPosition() - servoPosition, 0, 1);
            DbgLog.msg("ftc9773: deactivateButtonServo: cur position = %f, new position=%f",
                    buttonServoLinear.getPosition(), servoPosition);
            buttonServoLinear.setPosition(servoPosition);
            curOpMode.sleep(600);
        }
    }

    public void claimABeacon(double distanceFromWall) {
        double lengthToExtend = distanceFromWall - curLength;
        // lengthToExtend should be between 2 and strokeLength
        lengthToExtend =  Math.max(lengthToExtend, 2.0);
        lengthToExtend = Math.min(lengthToExtend, strokeLength);

        double timeToExtend = lengthToExtend * (1000 / buttonServoSpeed);
        DbgLog.msg("ftc9773: timeToExtend=%f millis, lengthToExtend=%f cm",
                timeToExtend, lengthToExtend);
        activateButtonServo(timeToExtend + 200, lengthToExtend);
        curOpMode.sleep(100);
        deactivateButtonServo(timeToExtend, 5);
    }

    public void verifyBeaconColor(){
        colorSensor1.enableLed(false);
        curOpMode.telemetry.addData("red: ", "%s", Integer.toString(colorSensor1.red()));
        curOpMode.telemetry.addData("blue: ", "%s", Integer.toString(colorSensor1.blue()));
        curOpMode.telemetry.update();
        DbgLog.msg("ftc9773: red value = %d, blue value = %d",colorSensor1.red(),colorSensor1.blue());
        //DbgLog.msg("color number = %x", colorSensor1.getI2cAddress().get7Bit());
    }

    public void verifyBeaconServo() {
        activateButtonServo(1000, 5);
        deactivateButtonServo(1000, 5);
    }

    public boolean isBeaconRed() {
        int redValue = colorSensor1.red();
        int blueValue = colorSensor1.blue();
        if (redValue > blueValue) {
            return (true);
        } else {
            return (false);
        }
//        if (redValue >= 3 && ((redValue - blueValue) >= 2)) {
//            return true;
//        } else {
//            return false;
//        }
    }

    public boolean isBeaconBlue() {
        int redValue = colorSensor1.red();
        int blueValue = colorSensor1.blue();
//        return (blueValue >= 3 && ((blueValue - redValue) >= 2));
        if (blueValue > redValue) {
            return (true);
        } else {
            return (false);
        }
    }

    public String checkBeaconColor() {
        DbgLog.msg("ftc9773: red=%d, blue=%d, green=%d", colorSensor1.red(), colorSensor1.blue(),
                colorSensor1.green());
        return null;
    }

    public void setBeaconStatus() {
        beaconColor = (isBeaconBlue() ? BeaconColor.BLUE :
                (isBeaconRed() ? BeaconColor.RED : BeaconColor.NONE));

        DbgLog.msg("ftc9773: Beacon color values: red=%d, blue=%d", colorSensor1.red(), colorSensor1.blue());
        DbgLog.msg("ftc9773: Beacon color detected: %s", (beaconColor == BeaconColor.RED) ? "red" :
                ((beaconColor == BeaconColor.BLUE) ? "blue" : "none"));
    }

    public BeaconColor getBeaconColor() {
        return (beaconColor);
    }

    public String getBeaconColorString() {
        if (beaconColor == BeaconColor.RED)
            return "red";
        else if (beaconColor == BeaconColor.BLUE)
            return "blue";
        else
            return "none";
    }

    public double getStrokeLength() {
        return strokeLength;
    }

    public int getBlue() { return (colorSensor1!=null ? colorSensor1.blue() : 0);}

    public int getRed() { return (colorSensor1!=null ? colorSensor1.red() : 0);}

}
