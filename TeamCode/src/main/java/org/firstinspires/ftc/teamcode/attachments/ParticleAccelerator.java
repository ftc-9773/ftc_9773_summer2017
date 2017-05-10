/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.MotorSpecsReader;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by michaelzhou on 11/13/16.
 */

public class ParticleAccelerator implements Attachment{
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor launcherMotor1=null, launcherMotor2=null;
    double motorPower=0.0;
    double motorCPR;
    double motorGearRatio;
    long rampUpTime = 2000; // default value in milli seconds
    Instrumentation.PartAccInstrumentor partAccInstr=null;
    boolean addedPartAccInstr=false;

    public ParticleAccelerator(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key=null;
        JSONObject launcherObj = null;
        JSONObject motorsObj = null, launcherMotorObj1 = null, launcherMotorObj2 = null;
        MotorSpecsReader motorSpecs;
        String runMode = null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "ParticleAccelerator");
            launcherObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(launcherObj, "motors");
            motorsObj = launcherObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        try{
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "partAccMotor1");
            launcherMotorObj1 = motorsObj.getJSONObject(key);
            launcherMotor1 = curOpMode.hardwareMap.dcMotor.get("partAccMotor1");
            if (launcherMotorObj1.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the launcher motor");
                launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "rampUpTime");
            rampUpTime = motorsObj.getLong(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "motorPower");
            motorPower = motorsObj.getDouble(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "runMode");
            runMode = motorsObj.getString(key);
            if (runMode.equalsIgnoreCase("RUN_USING_ENCODER"))
                launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else
                launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            int maxSpeed = launcherMotorObj.getInt("maxSpeed");
//            launcherMotor.setMaxSpeed(maxSpeed);
            // Set the zero power behaviour to float sot hat the motor stops gradually
            // This is recommended for high speed low torque motors
            launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // Get motorCPR value
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "motor");
            String motor1Type = motorsObj.getString(key);
            motorSpecs = new MotorSpecsReader(JsonReader.motorSpecsFile, motor1Type);
            motorCPR = motorSpecs.getCPR();
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "gearRatio");
            motorGearRatio = motorsObj.getDouble(key);

            DbgLog.msg("ftc9773: created launcher motor 1");
        } catch (JSONException e) {
            DbgLog.error("ftc9773: JSONException occurred! key = %s", key);
            e.printStackTrace();
        }
        try{
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "partAccMotor2");
            launcherMotorObj2 = motorsObj.getJSONObject(key);
            launcherMotor2 = curOpMode.hardwareMap.dcMotor.get("partAccMotor2");
            if (launcherMotorObj2.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the launcher motor");
                launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (runMode.equalsIgnoreCase("RUN_USING_ENCODER"))
                launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            else
                launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            int maxSpeed = launcherMotorObj.getInt("maxSpeed");
//            launcherMotor.setMaxSpeed(maxSpeed);
            // Set the zero power behaviour to float sot hat the motor stops gradually
            // This is recommended for high speed low torque motors
            launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DbgLog.msg("ftc9773: created launcher motor 2");
        } catch (JSONException e) {
            e.printStackTrace();
        }

        // Add partAccInstrumentation
        if (robot.instrLevel != Instrumentation.InstrumentationLevel.NONE) {
            if (robot.instrLevel == Instrumentation.InstrumentationLevel.COMPLETE)
                partAccInstr = robot.instrumentation.new PartAccInstrumentor(this, launcherMotor1, launcherMotor2, true);
            else
                partAccInstr = robot.instrumentation.new PartAccInstrumentor(this, launcherMotor1, launcherMotor2, false);
        }
        addedPartAccInstr = false;
    }

    public void activateParticleAccelerator() {
        // Ramp up the power gradually.
        // This is recommended for high speed low torque motors
//        ElapsedTime rampUpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        rampUpTimer.reset();
//        while ((rampUpTimer.milliseconds() < rampUpTime) && curOpMode.opModeIsActive()) {
//            launcherMotor.setPower(1 - (rampUpTimer.milliseconds() / rampUpTime));
//        }
        if (launcherMotor1 != null) {  launcherMotor1.setPower(motorPower); }
        if (launcherMotor2 != null) { launcherMotor2.setPower(motorPower); }
        if ((partAccInstr != null) && !addedPartAccInstr) {
            addedPartAccInstr = true;
            robot.instrumentation.addAction(partAccInstr);
            partAccInstr.reset();
        }
    }

    public void deactivateParticleAccelerator() {
        // Zero power behaviour was set to FLOAT in the constructor.
        if (launcherMotor1 != null) {  launcherMotor1.setPower(0.0); }
        if (launcherMotor2 != null) { launcherMotor2.setPower(0.0); }
        if ((partAccInstr != null) && addedPartAccInstr) {
            addedPartAccInstr = false;
            robot.instrumentation.removeAction(partAccInstr);
            curOpMode.telemetry.addData("shooter status:", "stopped");
            curOpMode.telemetry.update();
        }
    }

    public double getMotorCPR() {
        return motorCPR;
    }

    public double getMotorGearRatio() {
        return motorGearRatio;
    }

}
