/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ftcrobocracy on 1/31/17.
 */

public class BackgroundTasks {
    LinearOpMode curOpMode;
    FTCRobot robot;
    public enum TaskID {MONIOTR_YAW, BEACON_CLAIM}

    public List<BackgroundTasksBaseClass> backgrndObjects = new ArrayList<BackgroundTasksBaseClass>();

    public class BackgroundTasksBaseClass {
        public TaskID taskID;
        public int iterationCount;
        public void startTask() {return;}
        public void resetTask() {return;}
        public void continueTask() {return;}
        public void endTask() {return;}
        public void printToConsole() {return;}
    }

    public class BeaconServoExtender extends BackgroundTasksBaseClass {
        private BeaconClaim beaconClaimObj;
        private ElapsedTime timer;
        private double targetLength, startingLength;
        private double timeout; // in milli seconds
        private BeaconClaim.BeaconClaimOperation operation;

        public BeaconServoExtender(BeaconClaim beaconClaimObj, double targetLength, double timeout) {
            this.beaconClaimObj = beaconClaimObj;
            this.timeout = timeout;
            this.targetLength = targetLength;
            this.operation = BeaconClaim.BeaconClaimOperation.NONE;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        public BeaconServoExtender(BeaconClaim beaconClaimObj, double timeout) {
            this.beaconClaimObj = beaconClaimObj;
            this.timeout = timeout;
            this.operation = BeaconClaim.BeaconClaimOperation.NONE;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        @Override
        public void startTask() {
            if (!robot.enableBackgroundTasks) { return; }
            timer.reset();
            this.startingLength = beaconClaimObj.getCurLength();
            if (startingLength < targetLength) {
                operation = BeaconClaim.BeaconClaimOperation.EXTEND;
                DbgLog.msg("ftc9773: Trying to extend the beacon servo from %f cm to %f cm ", startingLength, targetLength);
            } else if (startingLength > targetLength) {
                operation = BeaconClaim.BeaconClaimOperation.RETRACT;
                DbgLog.msg("ftc9773: Trying to retract the beacon servo from %f cm to %f cm ", startingLength, targetLength);
            } else {
                operation = BeaconClaim.BeaconClaimOperation.NONE;
            }
        }

        public void setTaskParams(double targetLength, double timeout) {
            this.targetLength = Range.clip(targetLength, 0, beaconClaimObj.getStrokeLength());
            this.timeout = timeout;
        }

        @Override
        public void continueTask() {
            if (!robot.enableBackgroundTasks) { return; }

            if (timer.milliseconds() < timeout) {
                if ((operation == BeaconClaim.BeaconClaimOperation.EXTEND) &&
                        (beaconClaimObj.getCurLength() <= targetLength)) {
                    beaconClaimObj.pushBeacon();
                    iterationCount++;
                } else if ((operation == BeaconClaim.BeaconClaimOperation.RETRACT) &&
                        (beaconClaimObj.getCurLength() >= targetLength)) {
                    beaconClaimObj.retractBeacon();
                    iterationCount++;
                } else {
                    beaconClaimObj.idleBeacon();
                }
            } else {
                beaconClaimObj.idleBeacon();
            }
        }

        @Override
        public void endTask() {
            if (!robot.enableBackgroundTasks) { return; }
            beaconClaimObj.idleBeacon();
        }

        @Override
        public void resetTask() {
            if (!robot.enableBackgroundTasks) { return; }

            timer.reset();
            startingLength = targetLength = 0;
            timeout = 0;
            this.operation = BeaconClaim.BeaconClaimOperation.NONE;
        }

        @Override
        public void printToConsole() {
            if (!robot.enableBackgroundTasks) { return; }
            DbgLog.msg("ftc9773: startingLength=%f, targetLength=%f, timeout=%f, iterationCount=%d",
                    startingLength, targetLength, timeout, iterationCount);
        }
    }

    public BackgroundTasks(FTCRobot robot, LinearOpMode curOpMode) {
        this.robot = robot;
        this.curOpMode = curOpMode;
    }

}
