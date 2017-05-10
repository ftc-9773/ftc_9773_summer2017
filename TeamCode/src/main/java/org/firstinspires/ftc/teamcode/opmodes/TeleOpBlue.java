/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {
    @Override
    public void runOpMode(){
        FTCRobot robot;
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                "TeleOpBlue.json");
        String robotName = null;
        boolean printDbgMsg = false;
        String instrLevelStr=null;

        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }
        printDbgMsg = opmodeCfg.getBooleanValueForKey(opmodeCfg.jsonRoot, "printDebugMsg");
        instrLevelStr = opmodeCfg.getStringValueForKey(opmodeCfg.jsonRoot, "InstrumentationLevel");

        // TODO: 12/31/16 Instead of passing a 3rd parameter Autonomous/Teleop, use annotations to detect
        //     that FTCRobot is being instantiated for Autonomous or Teleop mode
        robot = new FTCRobot(this, robotName, "Teleop", opmodeCfg);
        robot.runTeleOp("blue");
    }
}
