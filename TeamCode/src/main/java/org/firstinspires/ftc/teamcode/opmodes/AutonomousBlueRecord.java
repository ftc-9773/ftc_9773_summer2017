/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@Autonomous(name = "AutonomousBlueRecord", group = "TeleOp")
public class AutonomousBlueRecord extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir + "AutonomousBlueRecord.json");
        String robotName = null;
        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }
        // TODO: 12/31/16 Instead of passing a 3rd parameter Autonomous/Teleop, use annotations to detect
        //     that FTCRobot is being instantiated for Autonomous or Teleop mode
        FTCRobot robot = new FTCRobot(this, robotName, "Autonomous", opmodeCfg);
        robot.autonomousRecord(opmodeCfg, "blue");
    }
}
