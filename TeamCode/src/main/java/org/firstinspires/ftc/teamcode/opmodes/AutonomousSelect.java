package org.firstinspires.ftc.teamcode.opmodes;

import android.util.JsonWriter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutonomousActions;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.JsonReaders.AutonomousOptionsReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.*;

/**
 * Created by michaelzhou on 3/26/17.
 */

@TeleOp(name = "Autonomous Select", group = "TeleOp")
public class AutonomousSelect extends LinearOpMode {
    FTCRobot robot;
    String alliance;
    String autonomousOption;
    AutonomousOptionsReader jsonReader;
    List<String> autonomousOptions;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            alliance = redOrBlue();
            telemetry.addData("Final selected alliance: ", alliance);
            telemetry.update();
            sleep(500);
            allOptions();
        } catch (JSONException e) {
            e.printStackTrace();
        }

    }

    public String redOrBlue(){
        alliance = "red";
        waitForStart();

        telemetry.addData("Current alliance", alliance);
        telemetry.update();

        while(opModeIsActive()) {
            if(gamepad1.dpad_down){
                alliance = "blue";
                telemetry.addData("Current Alliance", alliance);
                telemetry.update();
            }
            if(gamepad1.dpad_up){
                alliance = "red";
                telemetry.addData("Current ALLiance", alliance);
                telemetry.update();
            }
            else if(gamepad1.a){
                return alliance;
            }
            idle();
        }
        return alliance;
    }

    public void allOptions() throws JSONException {
        DbgLog.msg("ftc9773: Reached here 0!");
        int index = 0;
        jsonReader = new AutonomousOptionsReader(JsonReader.autonomousOptFile);
        DbgLog.msg("ftc9773: Reached here 1!");
        autonomousOptions = jsonReader.getAll();
        DbgLog.msg("ftc9773: Reached here 2!");
        DbgLog.msg("ftc9773: autonomousOptions length = %d", autonomousOptions.size());
        autonomousOption = autonomousOptions.get(index);
        DbgLog.msg("ftc9773: Reached here 3!");

        telemetry.addData("Current autonomous option", autonomousOption);
        telemetry.update();
        DbgLog.msg("ftc9773: Option: %s", autonomousOption);

        while(opModeIsActive()) {
            if(gamepad1.dpad_down){
                index = (index+1) < autonomousOptions.size() ? (index+1) : 0;
                autonomousOption = autonomousOptions.get(index);
                telemetry.addData("Current autonomous option", autonomousOption);
                telemetry.update();
                DbgLog.msg("ftc9773: Current autonomous option: %s, index=%d", autonomousOption, index);
            }
            else if (gamepad1.dpad_up){
                index = (index-1) >= 0 ? (index-1) : (autonomousOptions.size() -1);
                autonomousOption = autonomousOptions.get(index);
                telemetry.addData("Current autonomous option", autonomousOption);
                telemetry.update();
                DbgLog.msg("ftc9773: Current autonomous option: %s, index=%d", autonomousOption, index);
            }
            else if(gamepad1.a){
                DbgLog.msg("ftc9773: gamepad1.a is selected");
                writeToFile(JsonReader.opModesDir + (alliance.equals("red") ? "AutonomousRed.json": "AutonomousBlue.json"), autonomousOption);
                return;
            }
            sleep(300);
        }
    }

//    private List<String> searchForColor(String color){
//        for(String option: autonomousOptions) {
//            if (!option.toLowerCase().contains(color.toLowerCase())) {
//                autonomousOptions.remove(option);
//            }
//        }
//    }
//
    //TODO: DO we need to remove and put to replace, or just put the value in?
    private void writeToFile(String path, String autonomousOption) throws JSONException {
        JsonReader reader = new JsonReader(path);
        String option = reader.jsonRoot.getString("autonomousOption");
        String newString = reader.jsonStr.replace(option, autonomousOption).replace(",", ",\n").replace("}","\n}");
        FileRW filerw = new FileRW(path, true);
        filerw.fileWrite(newString);
        filerw.close();
        DbgLog.msg("ftc9773: Input: %s",autonomousOption);
        DbgLog.msg("ftc9773: Result: %s",reader.jsonRoot.getString("autonomousOption"));
        return;
//        try(FileWriter writer = new FileWriter(path)){
//
//        }catch(Exception e){
//            e.printStackTrace();
//        }
//        robot = new FTCRobot(this, reader.jsonRoot.getString("robot"), "Autonomous");
//        robot.runAutonomous(autonomousOption, alliance, reader.jsonRoot.getLong("startingDelay"), reader.jsonRoot.getInt("startingPosition"), reader.jsonRoot.getBoolean("enableBackGroundTasks"));
    }

}
