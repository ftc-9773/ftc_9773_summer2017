/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.navigation.NavigationChecks;
import org.firstinspires.ftc.teamcode.util.JsonReaders.DriveSysReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.MotorSpecsReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.WheelSpecsReader;

import java.lang.annotation.Annotation;


public abstract class DriveSystem {
    public enum DriveSysType {TWO_MOTOR_DRIVE, FOUR_MOTOR_6WD}
    LinearOpMode curOpMode;
    FTCRobot robot;
    DriveSysType driveSysType;

    public interface ElapsedEncoderCounts {
        void reset();
        double getDistanceTravelledInInches();
        double getDegreesTurned();
        void printCurrentEncoderCounts();
        void copyFrom(ElapsedEncoderCounts otherElapsedCounts);
    }

    public interface DriveSysPosition {
        void savePostion();
        void resetPosition();
        void driveToPosition(double speed);
        void driveToMidPosition(DriveSysPosition driveSysPosition, double speed);
        double getDistanceFromCurPosition();
    }

    public DriveSystem() {
    }

    public static DriveSystem createDriveSystem(LinearOpMode curOpMode, FTCRobot robot,
                                                String driveSysName) {
        DriveSystem driveSys = null;
        DriveSysReader driveSysReader = new DriveSysReader(JsonReader.driveSystemsFile, driveSysName);
        DbgLog.msg("ftc9773: driveSysName=%s", driveSysReader.getDriveSysName());
        WheelSpecsReader wheelSpecs = new WheelSpecsReader(JsonReader.wheelSpecsFile,
                driveSysReader.getWheelType());
        if(driveSysName.equals("4Motor4WDSteering")||driveSysName.equals("4Motor6WDSteering")) {
            int CPR = 0;
            double wheelDiameter = 0.0;
            double frictionCoeff = 1.0;
            FourMotorSteeringDrive fourMotorSteeringDrive;

            // Assume that all Drive System motors are of same type,
            // and get the CPR just for one motor
            String fMotorL_type = driveSysReader.getMotorType("fMotorL");
            String wheel_type = driveSysReader.getWheelType();
            MotorSpecsReader motorSpecs =
                    new MotorSpecsReader(JsonReader.motorSpecsFile, fMotorL_type);
            CPR = motorSpecs.getCPR();
            wheelDiameter = wheelSpecs.getDiameter();
            frictionCoeff = wheelSpecs.getFrictionCoeff();

            DcMotor fMotorL = curOpMode.hardwareMap.dcMotor.get("fMotorL");
            DcMotor fMotorR = curOpMode.hardwareMap.dcMotor.get("fMotorR");
            DcMotor rMotorL = curOpMode.hardwareMap.dcMotor.get("rMotorL");
            DcMotor rMotorR = curOpMode.hardwareMap.dcMotor.get("rMotorR");

            Wheel wheel = new Wheel(wheel_type, wheelDiameter);

            DbgLog.msg("ftc9773: wheel diameter = %f", wheel.diameter);
            // Determine if autonomous or teleop
            int maxSpeedCPS=4000; // = driveSysReader.getMaxMotorSpeed();
            if (robot.autoOrTeleop.equalsIgnoreCase("Autonomous")) {
                maxSpeedCPS = driveSysReader.getMaxMotorSpeed("AutonomousMaxMotorSpeed");
            } else if (robot.autoOrTeleop.equalsIgnoreCase("Teleop")) {
                maxSpeedCPS = driveSysReader.getMaxMotorSpeed("TeleOpMaxMotorSpeed");
            }
            DbgLog.msg("ftc9773: maxSpeedCPS = %d", maxSpeedCPS);
            fourMotorSteeringDrive = new FourMotorSteeringDrive(fMotorL, rMotorL, fMotorR, rMotorR,
                    maxSpeedCPS, frictionCoeff, robot.distanceBetweenWheels, wheel, CPR, robot.autoOrTeleop);
            fourMotorSteeringDrive.curOpMode = curOpMode;
            fourMotorSteeringDrive.robot = robot;
            fourMotorSteeringDrive.driveSysType = DriveSysType.FOUR_MOTOR_6WD;
            driveSys = (DriveSystem) fourMotorSteeringDrive;
        } else if (driveSysName.equalsIgnoreCase("2Motor2WDSteering")) {
            int CPR = 0;
            double wheelDiameter = 0.0;
            double frictionCoeff = 1.0;
            TwoMotorDrive twoMotorDrive;

            String motorL_type = driveSysReader.getMotorType("motorL");
            String wheel_type = driveSysReader.getWheelType();
            MotorSpecsReader motorSpecs =
                    new MotorSpecsReader(JsonReader.motorSpecsFile, motorL_type);
            CPR = motorSpecs.getCPR();
            wheelDiameter = wheelSpecs.getDiameter();
            frictionCoeff = wheelSpecs.getFrictionCoeff();

            DcMotor motorL = curOpMode.hardwareMap.dcMotor.get("motorL");
            DcMotor motorR = curOpMode.hardwareMap.dcMotor.get("motorR");
            Wheel wheel = new Wheel(wheel_type, wheelDiameter);

            DbgLog.msg("ftc9773: wheel diameter = %f", wheel.diameter);
            // Determine if autonomous or teleop
            int maxSpeedCPS=4000; // = driveSysReader.getMaxMotorSpeed();
            if (robot.autoOrTeleop.equalsIgnoreCase("Autonomous")) {
                maxSpeedCPS = driveSysReader.getMaxMotorSpeed("AutonomousMaxMotorSpeed");
            } else if (robot.autoOrTeleop.equalsIgnoreCase("Teleop")) {
                maxSpeedCPS = driveSysReader.getMaxMotorSpeed("TeleOpMaxMotorSpeed");
            }
            DbgLog.msg("ftc9773: maxSpeedCPS = %d", maxSpeedCPS);
            twoMotorDrive = new TwoMotorDrive(motorL, motorR, maxSpeedCPS, frictionCoeff, wheel, CPR);
            twoMotorDrive.curOpMode = curOpMode;
            twoMotorDrive.robot = robot;
            twoMotorDrive.driveSysType = DriveSysType.TWO_MOTOR_DRIVE;
            driveSys = (DriveSystem) twoMotorDrive;
        }
        return (driveSys);
    }

    public void drive(float speed, float direction) {return;}
    public DriveSysType getDriveSysType() { return driveSysType;}

    public abstract void setZeroPowerMode(DcMotor.ZeroPowerBehavior zp_behavior);

    public abstract DcMotor.ZeroPowerBehavior getZeroPowerBehavior();

    public void driveToDistance(float speed, double distanceInInches){return;}
    public void turnOrSpin(double leftSpeed, double rightSpeed) {return;}
    public void stop() {return;}

    public abstract void turnDegrees(double degrees, float speed, NavigationChecks navExc);

    public abstract void driveToEncoderCounts(ElapsedEncoderCounts fromCounts, ElapsedEncoderCounts toCounts,
                                              float speed);

    public abstract void setMaxSpeedCPS(int maxSpeedCPS);
    public abstract void resumeMaxSpeed();
    public abstract void reverse();
    public abstract ElapsedEncoderCounts getNewElapsedCountsObj();
    public abstract DriveSysPosition getNewDrivesysPositionObj();
    public abstract void printCurrentPosition();
    public abstract void initForPlay();
    public abstract String getDriveSysInstrData();

    public void testEncoders(){return;}

    public abstract void scaleDrivePower(double multiplier);
    public abstract double getScaleDriveMultiplier();
    public abstract void scaleSpinPower(double multiplier);
    public abstract double getScaleSpinMultiplier();
    public abstract void reverseTeleop();
    public abstract void unreverseTeleop();

}
