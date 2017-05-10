/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousActions;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.navigation.LineFollow;
import org.json.JSONException;
import org.json.JSONObject;

@Autonomous(name = "AutonomousBlue", group = "Autonomous")
public class AutonomousBlue extends LinearOpMode {

    FTCRobot robot;

    @Override
    public void runOpMode() {
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir + "AutonomousBlue.json");
        String autonomousOpt = null;
        String robotName = null;
        long startingDelay = 0;
        int startingPosition = 1;
        boolean enableBackGroundTasks = false;
        try {
            autonomousOpt = opmodeCfg.jsonRoot.getString("autonomousOption");
            robotName = opmodeCfg.jsonRoot.getString("robot");
            startingDelay = opmodeCfg.jsonRoot.getLong("startingDelay");
            startingPosition = opmodeCfg.jsonRoot.getInt("startingPosition");
            enableBackGroundTasks = opmodeCfg.jsonRoot.getBoolean("enableBackGroundTasks");
        } catch (JSONException e) {
            e.printStackTrace();
        }


        // TODO: 12/31/16 Instead of passing a 3rd parameter Autonomous/Teleop, use annotations to detect
        //     that FTCRobot is being instantiated for Autonomous or Teleop mode
        robot = new FTCRobot(this, robotName, "Autonomous", opmodeCfg);
        robot.runAutonomous(autonomousOpt, "blue", startingDelay, startingPosition, enableBackGroundTasks);
    }
}
