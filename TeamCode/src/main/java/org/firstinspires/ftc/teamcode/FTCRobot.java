/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.attachments.Attachment;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.attachments.CapBallLift;
import org.firstinspires.ftc.teamcode.attachments.Harvester;
import org.firstinspires.ftc.teamcode.attachments.ParticleAccelerator;
import org.firstinspires.ftc.teamcode.attachments.ParticleRelease;
import org.firstinspires.ftc.teamcode.attachments.WallFollower;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.navigation.CoordinateSys;
import org.firstinspires.ftc.teamcode.navigation.Navigation;
import org.firstinspires.ftc.teamcode.util.BackgroundTasks;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.RobotConfigReader;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.TimeUnit;

/**
 * Top-level robot class. This class contains all attachments, and calls both the TeleOp and Autonomous initialize/start
 * methods/sequences. An object of this class is created in every opmode.
 *
 * @author pranavburugula
 * @version 2.2
 */
public class FTCRobot {
    public LinearOpMode curOpMode;
    public DriverStation driverStation;
    public DriveSystem driveSystem=null;
    public Navigation navigation =null;
    private Attachment[] attachmentsArr;
    public AutonomousActions autonomousActions;
    public BeaconClaim beaconClaimObj;
    public WallFollower wallFollowObj;
    public CapBallLift capBallLiftObj;
    public Harvester harvesterObj;
    public ParticleAccelerator partAccObj;
    public ParticleRelease partRelObj;
    public double distanceLeft;
    public double distanceRight;
    public double distanceBetweenWheels;
    public String autoOrTeleop;
    public Instrumentation instrumentation;
    public BackgroundTasks backgroundTasks;
    public boolean enableBackgroundTasks=false;
    public boolean printDebugMsg=false;
    public Instrumentation.InstrumentationLevel instrLevel= Instrumentation.InstrumentationLevel.SUMMARY;

    /**
     * Reads robots JSON file, initializes drive system and attachments.
     * @param curOpMode Current running opmode, used for FTC libraries.
     * @param robotName Name of the robot configuration to initialize.
     * @param autoOrTeleop
     */
    public FTCRobot(LinearOpMode curOpMode, String robotName, String autoOrTeleop,
                    JsonReader opmodeCfg) {
        this.curOpMode = curOpMode;
        this.driverStation = new DriverStation(this, curOpMode);
        this.autoOrTeleop = autoOrTeleop;
        this.printDebugMsg = opmodeCfg.getBooleanValueForKey(opmodeCfg.jsonRoot, "printDebugMsg");
        String instrLevelStr = opmodeCfg.getStringValueForKey(opmodeCfg.jsonRoot, "InstrumentationLevel");
        if (instrLevelStr.equalsIgnoreCase("none")) {
            instrLevel = Instrumentation.InstrumentationLevel.NONE;
        } else if (instrLevelStr.equalsIgnoreCase("summary")) {
            instrLevel = Instrumentation.InstrumentationLevel.SUMMARY;
        } else {
            instrLevel = Instrumentation.InstrumentationLevel.COMPLETE;
        }

        RobotConfigReader robotConfig;
        robotConfig = new RobotConfigReader(JsonReader.baseDir+"robots.json",  robotName);
        String driveSysName = null;
//        distanceLeft = robotConfig.getDistanceLeft();
//        distanceRight = robotConfig.getDistanceRight();
        distanceLeft = distanceRight = 0;
        distanceBetweenWheels = robotConfig.getDistanceBetweenWheels();
        DbgLog.msg("ftc9773: distanceBetweenWheels=%f", distanceBetweenWheels);

        // Initialize the Instrumentation object
        if (instrLevel != Instrumentation.InstrumentationLevel.NONE) {
            instrumentation = new Instrumentation(this, curOpMode, robotConfig.getString("loopRuntimeLog"),
                    robotConfig.getString("rangeSensorLog"), robotConfig.getString("gyroLog"),
                    robotConfig.getString("odsLog"), robotConfig.getString("colorLog"), robotConfig.getString("partAccLog"));
            DbgLog.msg("ftc9773: Initialized the Instrumentation object");
        } else {
            DbgLog.msg("ftc9773: Instrumentation has been disabled");
        }

        // Initialize the BackgroundTasks object
        backgroundTasks = new BackgroundTasks(this, curOpMode);

        // Instantiate the Drive System
        try {
            driveSysName = robotConfig.getDriveSysName();
            if (driveSysName != null) {
                DbgLog.msg("ftc9773: driveSysName = %s", driveSysName);
                driveSystem = DriveSystem.createDriveSystem(curOpMode, this, driveSysName);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        if (driveSystem == null) {
            DbgLog.error("ftc9773: Drivesystem not properly initialized");
        }

        // Create the objects for attachments
        createAttachments(robotConfig.getAttachments(autoOrTeleop));

        // Initialize navigation subsystem
        if (robotConfig.getNavigationOption(autoOrTeleop) != null) {
            navigation = new Navigation(this, curOpMode, robotConfig.getNavigationOption(autoOrTeleop));
        } else{
            navigation = null;
        }

        DbgLog.msg("ftc9773: Done with robot initialization.  Current Voltage = %f", getVoltage());
        DbgLog.msg("ftc9773: Applications external storage directory=%s",
                curOpMode.hardwareMap.appContext.getExternalFilesDir(null));
    }

    /**
     * Iterates through listed attachments in robots JSON file, and initializes.
     * @param attachments   Array of attachment names specified in robots JSON file
     */
    private void createAttachments(String[] attachments) {
        if (attachments == null) {
            return;
        }

        JsonReader attachmentsReader = new JsonReader(JsonReader.attachments);
        JSONObject rootObj = attachmentsReader.jsonRoot;
        attachmentsArr = new Attachment[attachments.length];
        for (int i=0; i<attachments.length; i++) {
            switch (attachments[i]) {
                case "BeaconClaim":
                    attachmentsArr[i] = new BeaconClaim(this, curOpMode, rootObj, attachments[i]);
                    beaconClaimObj = (BeaconClaim) attachmentsArr[i];
                    DbgLog.msg("ftc9773: beaconClaimObj created for Autonomous");
                    break;
                case "BeaconClaimTeleOp":
                    attachmentsArr[i] = new BeaconClaim(this, curOpMode, rootObj, attachments[i]);
                    beaconClaimObj = (BeaconClaim) attachmentsArr[i];
                    DbgLog.msg("ftc9773: beaconClaimObj created for TeleOp");
                    break;
                case "WallFollower" :
                    attachmentsArr[i] = new WallFollower(this, curOpMode, rootObj);
                    wallFollowObj = (WallFollower) attachmentsArr[i];
                    DbgLog.msg("ftc9773: wallFollowObj created");
                    break;
                case "CapBallLift":
                    attachmentsArr[i] = new CapBallLift(this, curOpMode, rootObj);
                    capBallLiftObj = (CapBallLift) attachmentsArr[i];
                    driverStation.partAccStateMachine.switchState("Closed");
                    DbgLog.msg("ftc9773: capBallLiftObj created");
                    break;
                case "Harvester":
                    attachmentsArr[i] = new Harvester(this, curOpMode, rootObj);
                    harvesterObj = (Harvester) attachmentsArr[i];
                    DbgLog.msg("ftc9773: harvesterObj created");
                    break;
                case "ParticleAccelerator":
                    attachmentsArr[i] = new ParticleAccelerator(this, curOpMode, rootObj);
                    partAccObj = (ParticleAccelerator) attachmentsArr[i];
                    driverStation.partAccStateMachine.switchState("Off");
                    DbgLog.msg("ftc9773: partAccObj created");
                    break;
                case "ParticleRelease":
                    attachmentsArr[i] = new ParticleRelease(this, curOpMode, rootObj);
                    partRelObj = (ParticleRelease) attachmentsArr[i];
                    DbgLog.msg("ftc9773: partRelObj created");
                    break;
            }
        }
    }

    public void runTeleOp(String allianceColor) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        curOpMode.waitForStart();
        timer.reset();
        while (curOpMode.opModeIsActive()) {
            driverStation.getNextCmd();
            instrumentation.addInstrData();
            if (timer.milliseconds() >= 10) {
                navigation.coordinateSys.updatePose();
                double x = navigation.coordinateSys.getX();
                double y = navigation.coordinateSys.getY();
                double angle = navigation.coordinateSys.getAngle();
                timer.reset();
                curOpMode.telemetry.addData("X: ", x);
                curOpMode.telemetry.addData("Y: ", y);
                curOpMode.telemetry.addData("Angle: ", angle);
                curOpMode.telemetry.update();
                DbgLog.msg("ftc9773: CurPose: X: %f, Y: %f, Angle: %f", x, y, angle);
            }
            curOpMode.idle();
        }
        instrumentation.closeLog();
    }

    /**
     * Main autonomous method. Run from Autonomous opmode classes.
     * <br/>
     * This method does not actually run the Autonomous oppmode, rather it calls {@link AutonomousActions#doActions()},
     * which parses the autonomous JSON configuration.
     * @param autonomousOpt Name of the autonomous configuration as listed in the autonomous_options JSON file.
     * @param allianceColor Current alliance color.
     * @param startingDelay Amount of time, in seconds, the robot should wait before starting autonomous opmode. Used
     *                      to coordinate with alliance partner, and prevent collision.
     */
    public void runAutonomous(String autonomousOpt, String allianceColor,
                              long startingDelay, int startingPosition, boolean enableBackgroundTasks) {
        this.autonomousActions =
                new AutonomousActions(this, curOpMode, autonomousOpt, allianceColor);
        this.enableBackgroundTasks = enableBackgroundTasks;
        curOpMode.telemetry.update(); // one single update for all the data added

        try {
            curOpMode.waitForStart();
            DbgLog.msg("ftc9773: Starting delay = %d seconds", startingDelay);
            if (startingDelay > 0) curOpMode.sleep(startingDelay * 1000);
            navigation.initForPlay(); // Initialization after starting the robot
            driveSystem.initForPlay(); // Initialization of drive system after starting the robot
            autonomousActions.doActions();
            navigation.close();
            driveSystem.stop();
            curOpMode.stop();
        } catch (InterruptedException e) {
            navigation.close();
            driveSystem.stop();
            curOpMode.stop();
        }
    }

    /**
     * Record method for the Record/Replay system. Directly writes the record data (processed gamepad values)
     * into a .<!---->csv file.
     * @param opmodeCfg {@link JsonReader} used in the recording opmode class.
     * @param allianceColor Current alliance color.
     * @throws InterruptedException
     */
    public void autonomousRecord(JsonReader opmodeCfg, String allianceColor) throws InterruptedException {
        long clockCycle=5000;
        String recordFilePath, recordFileName=null;
        FileRW fileRW;
        String recordFilesDir=null;
        if (allianceColor.equalsIgnoreCase("red"))
            recordFilesDir = JsonReader.autonomousRedDir;
        else if (allianceColor.equalsIgnoreCase("blue"))
            recordFilesDir = JsonReader.autonomousBlueDir;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(opmodeCfg.jsonRoot, "recordFileName");
            recordFileName = opmodeCfg.jsonRoot.getString(key);
            clockCycle = opmodeCfg.jsonRoot.getLong("clock-cycle");
            clockCycle = clockCycle * 1000000; // convert milli seconds into nano seconds
        }
        catch (JSONException exc) {
            exc.printStackTrace();
        }
        recordFilePath = recordFilesDir + recordFileName;

        DbgLog.msg("ftc9773: clock Cycle = %d nanoseconds", clockCycle);
        DbgLog.msg("ftc9773: record filepath = %s", recordFilePath);

        fileRW = new FileRW(recordFilePath, true);
        // First row is a header row.
        String firstrow = "elapsedTimeNanoSec, speed, direction";
        fileRW.fileWrite(firstrow);

        curOpMode.waitForStart();
        long startingTime = System.nanoTime();
        long elapsedTime=0, prev_elapsedTime = 0;
        long sleepTime = 0;
        double spinAngle = 0;
        boolean isReverse = false;
        while (curOpMode.opModeIsActive()) {
            double speed = 0;
            if(!isReverse) {
                // ToDo: use navigation_options.json::"LineFollow_IMU_DriveSysEncoders"->
                //       "DriveSysEncoderVariables"->"StraightLineMaxSpeed" instead of
                //       multiplying with a hardcoded number 0.3
                speed = -curOpMode.gamepad1.left_stick_y * 0.3;
            }
            else if(isReverse){
                // ToDo: use navigation_options.json::"LineFollow_IMU_DriveSysEncoders"->
                //       "DriveSysEncoderVariables"->"StraightLineMaxSpeed" instead of
                //       multiplying with a hardcoded number 0.3
                speed = curOpMode.gamepad1.left_stick_y * 0.3;
            }
            // ToDo: use navigation_options.json::"LineFollow_IMU_DriveSysEncoders"->
            //       "DriveSysEncoderVariables"->"TurningMaxSpeed" instead of
            //       multiplying with a hardcoded number 0.5
            double direction = curOpMode.gamepad1.right_stick_x * 0.5;
            if(curOpMode.gamepad1.x){
                isReverse = true;
            }
            if(curOpMode.gamepad1.b){
                isReverse = false;
            }
            if(curOpMode.gamepad1.left_bumper){
                spinAngle = navigation.gyro.getYaw();
            }

            elapsedTime = System.nanoTime() - startingTime;
            sleepTime = clockCycle - (elapsedTime - prev_elapsedTime);
            if (sleepTime > 0) {
                TimeUnit.NANOSECONDS.sleep(sleepTime);
            }
            elapsedTime = System.nanoTime() - startingTime;
            driveSystem.drive((float) speed, (float) direction);
            if(spinAngle != 0) {
                fileRW.fileWrite(Long.toString(elapsedTime) + "," + Double.toString(speed) + "," +
                        Double.toString(direction) + "," + Double.toString(spinAngle));
                spinAngle = 0;
            }
            else {
                fileRW.fileWrite(Long.toString(elapsedTime) + "," + Double.toString(speed) + "," +
                        Double.toString(direction));
            }

//            DbgLog.msg(String.format("ftc9773: Speed: %f, Direction: %f", speed, direction));

            if(curOpMode.gamepad1.a){
                break;
            }

            DbgLog.msg("ftc9773: prev_elapsedTime=%d, elapsedTime=%d", prev_elapsedTime, elapsedTime);
            prev_elapsedTime = elapsedTime;
            // sleep(5);
        }
        DbgLog.msg("ftc9773: Is close executing?");
        fileRW.close();
    }

    /**
     * @return  Current battery voltage.
     */
    public double getVoltage() {
        return (curOpMode.hardwareMap.voltageSensor.iterator().next().getVoltage());
    }
}
