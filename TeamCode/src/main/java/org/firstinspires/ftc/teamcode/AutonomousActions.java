/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.JsonReaders.AutonomousOptionsReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.TimeUnit;

/**
 *
 */
public class AutonomousActions {
    FTCRobot robot;
    LinearOpMode curOpMode;
    public String allianceColor, otherColor;
    AutonomousOptionsReader autoCfg;
    String replayFilesDir;
    DriveSystem driveSystem;
    final double CM2INCHES = 0.3937;


    public AutonomousActions(FTCRobot robot, LinearOpMode curOpMode, String autoOption,
                             String allianceColor) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.driveSystem = robot.driveSystem;
        this.allianceColor = allianceColor;
        if (allianceColor.equalsIgnoreCase("red"))
            this.otherColor = "blue";
        else if (allianceColor.equalsIgnoreCase("blue"))
            this.otherColor = "red";
        curOpMode.telemetry.addData("autonomous option:", "%s", autoOption);
        autoCfg = new AutonomousOptionsReader(JsonReader.autonomousOptFile, autoOption);
        if (allianceColor.equalsIgnoreCase("Blue"))
            this.replayFilesDir = JsonReader.autonomousBlueDir;
        else if (allianceColor.equalsIgnoreCase("Red"))
            this.replayFilesDir = JsonReader.autonomousRedDir;
    }

    public void replayFileAction(String replayFile) throws InterruptedException {
        FileRW fileRW;
        fileRW = new FileRW(replayFile, false);
        fileRW.getNextLine(); // skip the header row
        String line;
        long startingTime = System.nanoTime();
        long elapsedTime = 0;
        long sleepTime = 0;
        int turnCounter = 0;
        while (((line = fileRW.getNextLine()) != null) && curOpMode.opModeIsActive()) {
            String[] lineElements = line.split(",");
//            DbgLog.msg("ftc9773: lineElements length = %d", lineElements.length);
            if (lineElements.length < 3) {
                continue;
            } else {
                long timestamp = Long.parseLong(lineElements[0]);
                double speed = Double.parseDouble(lineElements[1]);
                double direction = Double.parseDouble(lineElements[2]);
                elapsedTime = System.nanoTime() - startingTime;
                if (elapsedTime < timestamp) {
                    sleepTime = timestamp - elapsedTime;
                    TimeUnit.NANOSECONDS.sleep(sleepTime);
                }
                if (lineElements.length > 3) {
                    if (turnCounter == 0) {
                        DbgLog.msg("ftc9773: Yaw: %f, Target yaw = %s", robot.navigation.gyro.getYaw(), lineElements[3]);
                        robot.navigation.setRobotOrientation(Double.parseDouble(lineElements[3]),
                                robot.navigation.turnMaxSpeed);
                        DbgLog.msg("ftc9773: Reached target orientation");
                    }
                    turnCounter++;
                } else {
                    turnCounter = 0;
                    driveSystem.drive((float) speed, (float) direction);
                }
            }
        }
        fileRW.close();
    }

    private void claimCurrentBeacon() {
        if (robot.beaconClaimObj.beaconColor != BeaconClaim.BeaconColor.NONE) {
            // Get the range sensor value and pass it on to method
            double distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
            while (distanceFromWall >= 255) {
                curOpMode.sleep(20);
                distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
            }
            DbgLog.msg("ftc9773: claimCurrentBeacon(): Distance from wall = %f", distanceFromWall);
            robot.beaconClaimObj.claimABeacon(distanceFromWall);
        }
    }

    private String getStrFromActionObj(JSONObject actionObj, String key) {
        String value = null;
        try {
            key = JsonReader.getRealKeyIgnoreCase(actionObj, key);
            value = actionObj.getString(key);
        } catch (JSONException e) {
            DbgLog.error("ftc9773: getStrFromActionObj: cannot read the key %s", key);
            e.printStackTrace();
        }
        return (value);
    }

    private double getDoubleFromActionObj(JSONObject actionObj, String key) {
        double value = 0.0;
        try {
            key = JsonReader.getRealKeyIgnoreCase(actionObj, key);
            value = actionObj.getDouble(key);
        } catch (JSONException e) {
            DbgLog.error("ftc9773: getDoubleFromActionObj: cannot read the key %s", key);
            e.printStackTrace();
        }
        return (value);
    }

    private void
    gotoBeaconSide1(double fwDegrees, double bwDegrees, double motorSpeed, String robotDirection,
                    double ODSfrontExtraDistFw, double ODSbackExtraDistFw,
                    double ODSfrontExtraDistBw, double ODSbackExtraDistBw) {
        DbgLog.msg("ftc9773: gotoBeaconSide1(): fwDegrees=%f, bwDegrees=%f, motorSpeed=%f, robotDirection=%s, " +
                        "ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, ODSfrontExtraDistBw=%f, " +
                        "ODSbackExtraDistBw=%f",
                fwDegrees, bwDegrees, motorSpeed, robotDirection,
                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
        // 1: Go to till the first ODS hits the white line
        double degrees = 0;
        double additionalDistance = 0;
        String odsPosition = null;
        boolean driveBackwards = false;
        if (robotDirection.equalsIgnoreCase("forward")) {
            degrees = fwDegrees;
            driveBackwards = false;
            odsPosition = "front";
            additionalDistance = ODSfrontExtraDistFw;
        } else {
            degrees = bwDegrees;
            driveBackwards = true;
            odsPosition = "back";
            additionalDistance = ODSbackExtraDistBw;
        }
        DbgLog.msg("ftc9773: degrees=%f, driveBackwards=%b, odsPosition=%s, additionalDistance=%f",
                degrees, driveBackwards, odsPosition, additionalDistance);

        // 1. go until either front or back ODS sensor reaches the white line
        String odsSensorPosition = robot.navigation.goStraightToWhiteLine(degrees, (float)motorSpeed, driveBackwards,
                additionalDistance, "either");
        DbgLog.msg("ftc9773: White line detected by the %s ods sensor", odsSensorPosition);
        // 2. If the correct sensor sensed the white line, then drive additional distance.
        if (odsSensorPosition.equalsIgnoreCase(odsPosition))
            return;
        else {
            // 3. If the correct sensor did not sense the white line then go in the reverse direction
            //     and try again.
            DbgLog.msg("ftc9773: The first ODS sensor skipped the white line; so moving in the reverse direction");
            while (!odsSensorPosition.equalsIgnoreCase(odsPosition)) {
                double redoInches = driveBackwards ? 6 : -6;
                double redoDegrees = driveBackwards ? fwDegrees : bwDegrees;
                robot.navigation.goStraightToDistance(redoInches, redoDegrees, (float)motorSpeed);
                odsSensorPosition =
                        robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed,
                                driveBackwards, additionalDistance, "either");
                DbgLog.msg("ftc9773: White line detected by the %s ods sensor", odsSensorPosition);
            }
        }

        return;
    }

    private void
    goFromSide1ToSide2(double fwDegrees, double bwDegrees, double motorSpeed, String robotDirection,
                       double ODSfrontExtraDistFw, double ODSbackExtraDistFw,
                       double ODSfrontExtraDistBw, double ODSbackExtraDistBw) {
        DbgLog.msg("ftc9773: goFromSide1ToSide2(): fwDegrees=%f, bwDegrees=%f, motorSpeed=%f, robotDirection=%s, " +
                        "ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, ODSfrontExtraDistBw=%f, " +
                        "ODSbackExtraDistBw=%f",
                fwDegrees, bwDegrees, motorSpeed, robotDirection,
                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
        double degrees = 0;
        double additionalDistance = 0;
        String odsPosition = null;
        boolean driveBackwards = false;

        // other color
        if (robotDirection.equalsIgnoreCase("forward")) {
            degrees = fwDegrees;
            driveBackwards = false;
            odsPosition = "back";
            additionalDistance = ODSbackExtraDistFw;
        } else {
            degrees = bwDegrees;
            driveBackwards = true;
            odsPosition = "front";
            additionalDistance = ODSfrontExtraDistBw;
        }
        DbgLog.msg("ftc9773: degrees=%f, driveBackwards=%b, odsPosition=%s, additionalDistance=%f",
                degrees, driveBackwards, odsPosition, additionalDistance);
        String odsSensorPosition =
                robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed, driveBackwards,
                additionalDistance, odsPosition);
        DbgLog.msg("ftc9773: White line detected by the %s ods sensor", odsSensorPosition);

        return;
    }

    private void
    goFromSide2ToSide1(double fwDegrees, double bwDegrees, double motorSpeed, String robotDirection,
                       double ODSfrontExtraDistFw, double ODSbackExtraDistFw,
                       double ODSfrontExtraDistBw, double ODSbackExtraDistBw) {
        DbgLog.msg("ftc9773: goFromSide2ToSide1(): fwDegrees=%f, bwDegrees=%f, " +
                        "motorSpeed=%f, robotDirection=%s, ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, " +
                        "ODSfrontExtraDistBw=%f, ODSbackExtraDistBw=%f",
                fwDegrees, bwDegrees, motorSpeed, robotDirection,
                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
        double degrees = 0;
        double additionalDistance = 0;
        String odsPosition = null;
        boolean driveBackwards = false;
        if (robotDirection.equalsIgnoreCase("forward")) {
            odsPosition = "front";
            additionalDistance = ODSfrontExtraDistBw;
            driveBackwards = true;
            degrees = bwDegrees;
        } else {
            odsPosition = "back";
            additionalDistance = ODSbackExtraDistFw;
            driveBackwards = false;
            degrees = fwDegrees;
        }
        DbgLog.msg("ftc9773: degrees=%f, driveBackwards=%b, odsPosition=%s, additionalDistance=%f",
                degrees, driveBackwards, odsPosition, additionalDistance);
        String odsSensorPosition =
                robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed, driveBackwards,
                additionalDistance, odsPosition);
        DbgLog.msg("ftc9773: White line detected by the %s ods sensor", odsSensorPosition);

        return;
    }

    public void invokeMethod(String methodName, JSONObject actionObj) {
        switch (methodName) {
            case "claimAbeacon": {
                claimCurrentBeacon();
                break;
            }
            case "setBeaconStatus": {
                robot.beaconClaimObj.setBeaconStatus();
                break;
            }
            case "verifyBeaconColor": {
                robot.beaconClaimObj.verifyBeaconColor();
                break;
            }
            case "verifyBeaconServo": {
                robot.beaconClaimObj.verifyBeaconServo();
                break;
            }
            case "printNavigationValues": {
                robot.navigation.printNavigationValues();
                break;
            }
            case "TurnDegrees": {
                double degrees = 0.0;
                double speed = robot.navigation.turnMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                //robot.navigation.navxMicro.turnRobot(degrees, speed, navigationChecks);
                double curYaw = robot.navigation.gyro.getYaw();
                double targerYaw = robot.navigation.getTargetYaw(curYaw, degrees);
                DbgLog.msg("ftc9773: currentYaw = %f, degrees=%f, speed=%f, targetYaw=%f",
                        curYaw, degrees, speed, targerYaw);
                robot.navigation.setRobotOrientation(targerYaw, speed);
                break;
            }
            case "TurnUntilWhiteLine": {
                robot.navigation.lf.turnUntilWhiteLine(false);
                break;
            }
            case "DriveToDistance": {
                double distance = 0.0;
                double speed = robot.navigation.straightDrMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    distance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: DriveToDistance: inches=%f, motorSpeed=%f", distance, speed);
                robot.driveSystem.driveToDistance((float) speed, distance);
                break;
            }
            case "SetRobotOrientation": {
                double orientation = 0.0;
                double speed = robot.navigation.turnMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    orientation = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: SetRobotOrientation: angle=%f, speed=%f", orientation, speed);
                robot.navigation.setRobotOrientation(orientation, speed);
                break;
            }
            case "printMinMaxLightDetected": {
                double millis=5000;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "milliSeconds");
                    millis = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.lf.printMinMaxLightDetected(millis);
                break;
            }
            case "reverseDriveSystem": {
                driveSystem.reverse();
                break;
            }
            case "Sleep": {
                int milliseconds = 0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "milliSeconds");
                    milliseconds = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                curOpMode.sleep(milliseconds);
                break;
            }
            case "DriveAndClaimAllianceBeacon": {
                double motorSpeed = 0.0;
                double degrees = 0.0;
                boolean driveBackwards = false;
                double additionalDistance = 0.0;
                String driveToPosition = "last";
                String frontOrBackODS = "back";
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "additionalDistance");
                    additionalDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveToPosition");
                    driveToPosition = actionObj.getString(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "odsPosition");
                    frontOrBackODS = actionObj.getString(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: degrees=%f, motorSpeed=%f, driveBackwards=%b, additionalDistance=%f," +
                                "driveToPosition=%s, ODS position=%s", degrees, motorSpeed, driveBackwards,
                        additionalDistance, driveToPosition, frontOrBackODS);
                robot.navigation.driveToAllianceBeaconWhileScanning(degrees, (float) motorSpeed,
                        driveBackwards, additionalDistance, driveToPosition, frontOrBackODS);
                // Get the range sensor value and pass it on to method
                double distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                while (distanceFromWall >= 255) {
                    curOpMode.sleep(20);
                    distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                }
                DbgLog.msg("ftc9773: Distance from wall = %f", distanceFromWall);
                robot.beaconClaimObj.claimABeacon(distanceFromWall);
                break;
            }
            case "startPartAcc":
                robot.partAccObj.activateParticleAccelerator();
                break;
            case "stopPartAcc":
                robot.partAccObj.deactivateParticleAccelerator();
                break;
            case "releaseParticles":
                robot.partRelObj.releaseParticles();
                break;
            case "keepParticles":
                robot.partRelObj.keepParticles();
                break;
            case "shootParticles": {
                int numParticles = 1;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "numberOfParticles");
                    numParticles = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: shootParticles: numberOfParticles to shoot = %d", numParticles);
                if (numParticles == 1) {
                    robot.partRelObj.releaseParticles();
                    curOpMode.sleep(1000);
                    robot.partRelObj.keepParticles();
                } else if (numParticles == 2) {
                    robot.partRelObj.releaseParticles();
                    curOpMode.sleep(2000);
                    robot.partRelObj.keepParticles();
                }
                break;
            }
            case "UnfoldWallFollower": {
                String key = null;
                double millis = 200;
                try {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "timeToPowerTheServo");
                    millis = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.wallFollowObj.activateWallFollwer(millis);
                break;
            }
            case "IdleWallFollower": {
                robot.wallFollowObj.idle();
                break;
            }
            case "FoldWallFollower": {
                String key = null;
                double millis = 200;
                try {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "timeToPowerTheServo");
                    millis = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.wallFollowObj.deactivateWallFollower(millis);
                break;
            }
            case "GoStraightToDistance": {
                double inches = 0.0;
                double motorSpeed = 0.0;
                double degrees = 0.0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    inches = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: Degrees: %f, inches: %f, motorSpeed: %f", degrees, inches, motorSpeed);
                robot.navigation.goStraightToDistance(inches, degrees, (float) motorSpeed);
                break;
            }
            case "GoStraightToWhiteLine": {
                double motorSpeed = 0.0;
                double degrees = 0.0;
                boolean driveBackwards = false;
                double additionalDistance = 0.0;
                String frontOrBackODS = "back";
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "additionalDistance");
                    additionalDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "odsPosition");
                    frontOrBackODS = actionObj.getString(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: degrees=%f, motorSpeed=%f, driveBackwards=%b, additionalDistance=%f," +
                                "ODS position=%s",
                        degrees, motorSpeed, driveBackwards, additionalDistance, frontOrBackODS);
                robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed, driveBackwards,
                        additionalDistance, frontOrBackODS);
                break;
            }
            case "GoStraightToWhiteLineIfNeeded": {
                double motorSpeed = 0.0;
                double degrees = 0.0;
                boolean driveBackwards = false;
                double additionalDistance = 0.0;
                String frontOrBackODS = "back";
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "additionalDistance");
                    additionalDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "odsPosition");
                    frontOrBackODS = actionObj.getString(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: degrees=%f, motorSpeed=%f, driveBackwards=%b, additionalDistance=%f," +
                                "ODS position=%s",
                        degrees, motorSpeed, driveBackwards, additionalDistance, frontOrBackODS);
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor))
                    return;
                else
                    robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed, driveBackwards,
                            additionalDistance, frontOrBackODS);
                break;
            }
            case "GoStraight_Claim": {
                double motorSpeed = getDoubleFromActionObj(actionObj, "motorSpeed");
                double fwDegrees = getDoubleFromActionObj(actionObj, "fwDegrees");
                double bwDegrees = getDoubleFromActionObj(actionObj, "bwDegrees");
                String robotDirection = getStrFromActionObj(actionObj, "robotDirection");
                double ODSfrontExtraDistFw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistFw");
                double ODSbackExtraDistFw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistFw");
                double ODSfrontExtraDistBw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistBw");
                double ODSbackExtraDistBw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistBw");
                DbgLog.msg("ftc9773: invokeMethod(): fwDegrees=%f, bwDegrees=%f, motorSpeed=%f, robotDirection=%s, " +
                                "ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, ODSfrontExtraDistBw=%f, " +
                                "ODSbackExtraDistBw=%f",
                        fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);

                gotoBeaconSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                // 2. Sleep for 100 millis
                curOpMode.sleep(100);
                // 3. Set the beacon color
                robot.beaconClaimObj.setBeaconStatus();
                // 4. Claim or move to the other half of the beacon as needed
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) {
                    // claim the beacon
                    claimCurrentBeacon();
                } else if (!robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase("none")) {
                    goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    // Sleep for 100 millis
                    curOpMode.sleep(100);
                    // Set the beacon color
                    robot.beaconClaimObj.setBeaconStatus();
                    if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor))
                        claimCurrentBeacon();
                }
                break;
            }
            case "GoStraight_Claim_Verify_Claim": {
                double motorSpeed = getDoubleFromActionObj(actionObj, "motorSpeed");
                double fwDegrees = getDoubleFromActionObj(actionObj, "fwDegrees");
                double bwDegrees = getDoubleFromActionObj(actionObj, "bwDegrees");
                String robotDirection = getStrFromActionObj(actionObj, "robotDirection");
                double ODSfrontExtraDistFw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistFw");
                double ODSbackExtraDistFw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistFw");
                double ODSfrontExtraDistBw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistBw");
                double ODSbackExtraDistBw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistBw");
                DbgLog.msg("ftc9773: invokeMethod(): fwDegrees=%f, bwDegrees=%f, motorSpeed=%f, robotDirection=%s, " +
                                "ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, ODSfrontExtraDistBw=%f, " +
                                "ODSbackExtraDistBw=%f",
                        fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                int beaconHalfClaimed = 1; // 1 for 1st half 2 for 2nd half
                gotoBeaconSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                // 2. Sleep for 100 millis
                curOpMode.sleep(100);
                // 3. Set the beacon color
                robot.beaconClaimObj.setBeaconStatus();
                // A. Claim or move to the other half of the beacon as needed
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) {
                    // claim the beacon
                    claimCurrentBeacon();
                    beaconHalfClaimed = 1;
                } else if (!robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase("none")) {
                    // go to the 2nd half
                    goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    // claim the beacon
                    claimCurrentBeacon();
                    beaconHalfClaimed = 2;
                }

                // B. Verify
                if (beaconHalfClaimed == 1) {
                    // go to the 2nd half
                    goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                } else if (beaconHalfClaimed == 2) {
                    goFromSide2ToSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                }
                // Sleep for 100 millis
                curOpMode.sleep(100);
                // Set the beacon color
                robot.beaconClaimObj.setBeaconStatus();
                boolean reclaimNeeded = false;
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor))
                    reclaimNeeded = false;
                else
                    reclaimNeeded = true;

                // C. Claim again if necessary
                if (reclaimNeeded) {
                    if (beaconHalfClaimed == 1) {
                        // Originally 1st half was claimed; we are now at 2nd half; so go back to 1st half
                        goFromSide2ToSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    } else if (beaconHalfClaimed == 2) {
                        // Originally 2nd half was claimed; we are now at 1st half; so go to the 2nd half
                        goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    }
                    // claim the beacon
                    claimCurrentBeacon();
                }
                break;
            }
            case "GoStraight_V_C_V_C": {
                double motorSpeed = getDoubleFromActionObj(actionObj, "motorSpeed");
                double fwDegrees = getDoubleFromActionObj(actionObj, "fwDegrees");
                double bwDegrees = getDoubleFromActionObj(actionObj, "bwDegrees");
                String robotDirection = getStrFromActionObj(actionObj, "robotDirection");
                double ODSfrontExtraDistFw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistFw");
                double ODSbackExtraDistFw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistFw");
                double ODSfrontExtraDistBw = getDoubleFromActionObj(actionObj, "ODSfrontExtraDistBw");
                double ODSbackExtraDistBw = getDoubleFromActionObj(actionObj, "ODSbackExtraDistBw");
                DbgLog.msg("ftc9773: invokeMethod(): fwDegrees=%f, bwDegrees=%f, motorSpeed=%f, robotDirection=%s, " +
                                "ODSfrontExtraDistFw=%f, ODSbackExtraDistFw=%f, ODSfrontExtraDistBw=%f, " +
                                "ODSbackExtraDistBw=%f",
                        fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);

                boolean side1IsOurColor = false, side2IsOurColor = false;
                int currentSide = 0;

                // 1. Verify
                gotoBeaconSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                currentSide = 1;
                curOpMode.sleep(100);
                robot.beaconClaimObj.setBeaconStatus();
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor))
                    side1IsOurColor = true;
                else
                    side1IsOurColor = false;
                // go from side 1 to side2
                goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                        ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                currentSide = 2;
                curOpMode.sleep(100);
                robot.beaconClaimObj.setBeaconStatus();
                if (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor))
                    side2IsOurColor = true;
                else
                    side2IsOurColor = false;

                // 2. Claim if necessary
                if (side1IsOurColor && side2IsOurColor) {
                    return; // No more work to do
                } else if (!side1IsOurColor && !side2IsOurColor) {
                    // Both sides are of other alliance's color; in this case,
                    // pressing any side will turn the beacon to our color.
                    // So no need to move the robot to claim the beacon
                    claimCurrentBeacon();
                } else {
                    // We are currently at side 2.
                    if (side1IsOurColor) {
                        goFromSide2ToSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                        currentSide = 1;
                        // claim the beacon
                        claimCurrentBeacon();
                    } else if (side2IsOurColor) {
                        // claim the beacon
                        claimCurrentBeacon();
                    }
                }

                // 3. Verify again
                if (currentSide == 1) {
                    curOpMode.sleep(100);
                    robot.beaconClaimObj.setBeaconStatus();
                    side1IsOurColor = (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) ?
                            true : false;
                    goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    currentSide = 2;
                    curOpMode.sleep(100);
                    robot.beaconClaimObj.setBeaconStatus();
                    side2IsOurColor = (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) ?
                            true : false;
                } else if (currentSide == 2) {
                    curOpMode.sleep(100);
                    robot.beaconClaimObj.setBeaconStatus();
                    side2IsOurColor = (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) ?
                            true : false;
                    goFromSide2ToSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                            ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                    currentSide = 1;
                    curOpMode.sleep(100);
                    robot.beaconClaimObj.setBeaconStatus();
                    side1IsOurColor = (robot.beaconClaimObj.getBeaconColorString().equalsIgnoreCase(allianceColor)) ?
                            true : false;
                }
                // 4. Reclaim if necessary
                if (side1IsOurColor && side2IsOurColor) {
                    return; // No more work to do
                } else {
                    if ((currentSide == 1 && side1IsOurColor) || (currentSide == 2 && side2IsOurColor)) {
                        claimCurrentBeacon();
                    } else if (currentSide == 1) {
                        // The robot is currently at side 1
                        goFromSide1ToSide2(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                        currentSide = 2;
                        claimCurrentBeacon();

                    } else {
                        // The robot is currently at side 2.
                        goFromSide2ToSide1(fwDegrees, bwDegrees, motorSpeed, robotDirection,
                                ODSfrontExtraDistFw, ODSbackExtraDistFw, ODSfrontExtraDistBw, ODSbackExtraDistBw);
                        currentSide = 1;
                        // claim the beacon
                        claimCurrentBeacon();
                    }
                }

                break;
            }

            case "GoStraightAlongWall": {
                double inches = 0.0;
                double motorSpeed = 0.0;
                double degrees = 0.0;
                double targetDistFromWall = 0.0;
                double wallKp = 0.5;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    inches = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "targetDistance");
                    targetDistFromWall = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "wallKp");
                    wallKp = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: Degrees: %f, inches: %f, motorSpeed: %f, targetDistance=%f," +
                        " wallKp=%f", degrees, inches, motorSpeed, targetDistFromWall, wallKp);
                robot.navigation.goStraightAlongTheWall(inches, degrees, (float) motorSpeed,
                        targetDistFromWall, wallKp);
                break;
            }
            case "navxGoStraightPID": {
                double degrees = 0;
                String termCondition = null;
                double inches = 0.0;
                double speed = 0.5;
                boolean driveUntilWhiteLine = false;
                boolean driveBackwards = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "endingCondition");
                    termCondition = actionObj.getString(key);
                    if (termCondition.equalsIgnoreCase("driveToDistance")) {
                        key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                        inches = actionObj.getDouble(key);
                    } else if (termCondition.equalsIgnoreCase("driveUntilWhiteLine")) {
                        driveUntilWhiteLine = true;
                    }
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: degrees=%f, inches=%f, driveBackwards=%b", degrees, inches, driveBackwards);
                if (driveUntilWhiteLine) {
                    while ((!robot.navigation.lf.BackODSonWhiteLine()) && robot.curOpMode.opModeIsActive()) {
                        robot.navigation.gyro.goStraightPID(driveBackwards, degrees, (float) speed);
                    }
                    driveSystem.stop();
                } else {
                    // drive to distance using navx go straight pid controller
                    DriveSystem.ElapsedEncoderCounts elapsedCounts =
                            driveSystem.getNewElapsedCountsObj();
                    elapsedCounts.reset();
                    while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                            robot.curOpMode.opModeIsActive()) {
                        robot.navigation.gyro.goStraightPID(driveBackwards, degrees, (float) speed);
                    }
                    driveSystem.stop();
                }
                break;
            }
            case "shiftRobot": {
                double shiftDistance = 0.0;
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                double startingYaw = 0.0;
                double endingYaw = 0.0;
                boolean returnToSamePos = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "shiftDistance");
                    shiftDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "startingYaw");
                    startingYaw = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "endingYaw");
                    endingYaw = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: shiftDistance=%f, moveDistance=%f, motorSpeed=%f, startingYaw=%f, endingYaw=%f, returnToSamePos=%b",
                        shiftDistance, moveDistance, motorSpeed, startingYaw, endingYaw, returnToSamePos);
                robot.navigation.shiftRobot(shiftDistance, moveDistance, motorSpeed, returnToSamePos, startingYaw, endingYaw);
                break;
            }
            case "shiftToWall": {
                double targetDistance = 0.0; // in cm
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                boolean returnToSamePos = false;
                double distanceFromWall; // in cm
                double distTolerance = 2.0; // in cm
                double distanceToShift = 0.0;
                double startingYaw = (allianceColor.equalsIgnoreCase("red") ? 0.0 : 180);
                ;
                double endingYaw = startingYaw;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "targetDistance");
                    targetDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "distTolerance");
                    distTolerance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "startingYaw");
                    startingYaw = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "endingYaw");
                    endingYaw = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                int i = 0;
                while ((distanceFromWall >= 255) && (i < 10)) { // Try for 200 milli seconds
                    curOpMode.sleep(20);
                    distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                }
                if (distanceFromWall >= 255) {
                    // get the value from the weighted average of the range sensor
                    distanceFromWall = robot.navigation.getRangeSensorRunningAvg();
                    if (distanceFromWall < 0) {
                        // Ensure that the robot does not move sideways when the distance from wall
                        // is unknown.
                        distanceFromWall = targetDistance;
                    }
                }
                if ((Math.abs(targetDistance - distanceFromWall) > distTolerance)) {
                    distanceToShift = CM2INCHES * (targetDistance - distanceFromWall);
                    DbgLog.msg("ftc9773: targetDistance=%f cm, moveDistance=%f, distanceFromWall=%f cm, " +
                                    "tolerance = %f cm, distanceToShift=%f inches, motorSpeed=%f, returnToSamePos=%b," +
                                    "startingYaw=%f, endingYaw=%f",
                            targetDistance, moveDistance, distanceFromWall, distTolerance,
                            distanceToShift, motorSpeed, returnToSamePos, startingYaw, endingYaw);
                    robot.navigation.shiftRobot(distanceToShift, moveDistance, motorSpeed, returnToSamePos, startingYaw, endingYaw);
                } else {
                    DbgLog.msg("ftc9773: No need to shift; current distance from wall is within the tolerance!");
                    DbgLog.msg("ftc9773: targetDistance=%f cm, moveDistance=%f, distanceFromWall=%f cm, tolerance = %f cm, distanceToShift=%f inches, motorSpeed=%f, returnToSamePos=%b",
                            targetDistance, moveDistance, distanceFromWall, distTolerance,
                            distanceToShift, motorSpeed, returnToSamePos);
                    // Still have to move forward, even though the robot does not shift sideways
                    robot.navigation.goStraightToDistance(moveDistance, startingYaw, (float) motorSpeed);
                }
                break;
            }
            case "testEncoders": {
                robot.driveSystem.testEncoders();
            }
        }
    }

    public void doActions() throws InterruptedException {
        int len = autoCfg.actions.length();
        JSONObject actionObj;

        String replayFile;
        String methodName;
//        DbgLog.msg("ftc9773: Number of autonomous actions = %d", len);
        for (int i = 0; i < len && curOpMode.opModeIsActive(); i++) {
            DbgLog.msg("ftc9773: i=%d", i);
            try {
                actionObj = autoCfg.getAction(i);
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "type");
                if (actionObj.getString(key).equalsIgnoreCase("Replay")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    replayFile = this.replayFilesDir + actionObj.getString(key);
                    DbgLog.msg("ftc9773: Replaying the file %s", replayFile);
                    replayFileAction(replayFile);
                } else if (actionObj.getString(key).equalsIgnoreCase("Programmed")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    methodName = actionObj.getString(key);
                    DbgLog.msg("ftc9773: Invoking method: %s", methodName);
                    invokeMethod(methodName, actionObj);
                }
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
    }


}
