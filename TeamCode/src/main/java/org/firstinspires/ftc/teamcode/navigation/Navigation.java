/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.BackgroundTasks;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.json.JSONObject;



public class Navigation {
    FTCRobot robot;
    LinearOpMode curOpMode;
    JSONObject navOptObj;
    public GyroInterface gyro;
    public LineFollow lf;
    public EncoderNavigation encoderNav;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public CoordinateSys coordinateSys;
    public double lfMaxSpeed=1.0, straightDrMaxSpeed=1.0, turnMaxSpeed=1.0;
    public double driveSysTeleopMaxSpeed=1.0;
//    private Instrumentation.LoopRuntime driveToDistInstr, driveTillWhitelineInstr, turnRobotInstr;
//    private Instrumentation.LoopRuntime driveTillBeaconInstr;
    private Instrumentation.GyroDegrees gyroDegreesInstr=null;
    private Instrumentation.RangeSensorDistance rangeInstr=null;
    private Instrumentation.ODSlightDetected odsInstr=null;
    private Instrumentation.ColorSensorInstr colorInstr=null;
    private boolean printEveryUpdate=false;
    public enum GyroType {NAVX_MICRO, MR_GYRO, BNO055IMU}


    public enum SpinDirection {CLOCKWISE, COUNTERCLOCKWISE, NONE}

    public Navigation(FTCRobot robot, LinearOpMode curOpMode, String navOptionStr) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        double rangeSensorRunningAvg=0.0;

        NavigationOptionsReader navOption = new NavigationOptionsReader(JsonReader.navigationFile,
                navOptionStr);
        this.navOptObj = navOption.jsonRoot;

        if (navOption.lineFollowerExists()) {
            this.lf = new LineFollow(robot, navOption.getLightSensorName(),
                    navOption.getLFvariableDouble("lowSpeed"),
                    navOption.getLFvariableDouble("highSpeed"),
                    navOption.getLFvariableDouble("timeOut"),
                    navOption.getLFvariableDouble("white"),
                    navOption.getLFvariableDouble("black"));
        }
        else {
            this.lf = null;
        }

        if (navOption.imuExists()) {
            if (navOption.getIMUType().equalsIgnoreCase("MRgyro")) {
                DbgLog.msg("ftc9773: instantiating MR gyro");
//                curOpMode.telemetry.addData("ftc9773:", "instantiating MR gyro");
//                curOpMode.telemetry.update();
                this.gyro = new MRGyro(robot,curOpMode, this, navOption.getIMUVariableDouble("angleTolerance"),
                        navOption.getIMUVariableDouble("straightPID_kp"),
                        navOption.getIMUVariableDouble("turnPID_kp"));
            }
            else if (navOption.getIMUType().equalsIgnoreCase("BNO055IMU")){
                DbgLog.msg("ftc9773: instantiating BNO055IMU");
                this.gyro = new BoschIMU(robot, curOpMode, this, navOption.getIMUVariableDouble("angleTolerance"),
                        navOption.getIMUVariableDouble("straightPID_kp"),
                        navOption.getIMUVariableDouble("turnPID_kp"));
            }
        }
        else {
            this.gyro = null;
        }

        if (navOption.rangeSensorExists()) {
            this.rangeSensor = curOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor1");
            rangeSensorRunningAvg = navOption.getRangeSensorRunningAvgWeight();
        }
         else {
            this.rangeSensor = null;
        }

        if (navOption.encoderVarsExist()) {
            this.lfMaxSpeed = navOption.getLineFollowMaxSpeed();
            this.straightDrMaxSpeed = navOption.getStraightLineMaxSpeed();
            this.turnMaxSpeed = navOption.getTurningMaxSpeed();
            this.driveSysTeleopMaxSpeed = navOption.getDoubleDriveSysEncVar("DriveSysTeleOpMaxSpeed");
            this.encoderNav = new EncoderNavigation(robot, robot.driveSystem, curOpMode, this);
        }

        if (navOption.coordinateSysExists()){
            double startingX = navOption.getStartingPositionX();
            double startingY = navOption.getStartingPositionY();
            double startingAngle = navOption.getStartingPositionAngle();

            this.coordinateSys = new CoordinateSys(robot, curOpMode, startingX, startingY, startingAngle);
        }


        // Instantiate the common instrumentation objects (declared as inner classes in Instrumentation class
        if (robot.instrLevel != Instrumentation.InstrumentationLevel.NONE) {
            if (robot.instrLevel == Instrumentation.InstrumentationLevel.COMPLETE)
                printEveryUpdate = true;
            else
                printEveryUpdate = false;
            if (gyro != null)
                gyroDegreesInstr = robot.instrumentation.new GyroDegrees(this.gyro, printEveryUpdate);
            if (rangeSensor != null)
                rangeInstr = robot.instrumentation.new RangeSensorDistance(this.rangeSensor, rangeSensorRunningAvg, printEveryUpdate);
            if (lf != null)
                odsInstr = robot.instrumentation.new ODSlightDetected(this.lf, printEveryUpdate);

            colorInstr = robot.instrumentation.new ColorSensorInstr(robot.beaconClaimObj, printEveryUpdate);
        }
//        driveToDistInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_TO_DISTANCE);
//        driveTillWhitelineInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_UNTIL_WHITELINE);
//        driveTillBeaconInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_TILL_BEACON);
//        turnRobotInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.TURN_ROBOT);
    }

    public void printRangeSensorValue() {
        if (rangeSensor != null) {
            DbgLog.msg("ftc9773: range sensor distance in cm = %f", rangeSensor.getDistance(DistanceUnit.CM));
        }
    }

    public double getRangeSensorRunningAvg() {
        if (rangeSensor!= null) {
            if (rangeInstr != null)
                return (rangeInstr.getRunningAvg());
        } else {
            DbgLog.error("ftc9773: Error! Range sensor does not exist!");
            return (-1);
        }
        return (-1);
    }

    public void printNavigationValues() {
        DbgLog.msg("ftc9773: encoderYaw=%f, gyroYaw=%f", encoderNav.getCurrentYaw(), gyro.getYaw());
        DbgLog.msg("ftc9773: gyroPitch=%f, RangeSensor value cm = %f, ods light detected=%f", gyro.getPitch(),
                rangeSensor.getDistance(DistanceUnit.CM), lf.lightSensorBack.getLightDetected());
        DbgLog.msg("ftc9773: Drive system Encoder values:");
        robot.driveSystem.printCurrentPosition();
    }

    /**
     * Initialize the navigation system just after pressing the play button.
     */
    public void initForPlay() {
        if (gyro != null) {
            gyro.initAfterStart();
        }
    }

    public void close() {
        DbgLog.msg("ftc9773: Closing all the file objects");
        if (gyroDegreesInstr != null) { gyroDegreesInstr.closeLog(); }
        if (rangeInstr != null) { rangeInstr.closeLog(); }
        if (odsInstr != null) { odsInstr.closeLog(); }
        if (colorInstr != null) { colorInstr.closeLog(); }
//        if (driveToDistInstr != null) { driveToDistInstr.closeLog(); }
//        if (driveTillWhitelineInstr != null) { driveTillWhitelineInstr.closeLog(); }
//        if (turnRobotInstr != null) { turnRobotInstr.closeLog(); }
        gyro.close();
    }

    public double distanceBetweenAngles(double angle1, double angle2) {
        // Both angle1 and angle2 are assumed to be positive numbers between 0 and 360
        // The returnValue is between 0 and 180.
        double angleDistance= Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }

        return (angleDistance);
    }

    /**
     * Calculates whether the robot has to spin clockwise or counter clockwise to go from
     * currentYaw to targetYaw
     * @param curYaw
     * @param targetYaw
     * @return CLOCKWISE, COUNTERCLOCKWISE, NONE
     */
    public SpinDirection getSpinDirection (double curYaw, double targetYaw) {
        SpinDirection direction = SpinDirection.NONE;
        double diffYaw = targetYaw - curYaw;

        double degreesToTurn = diffYaw>180 ? diffYaw-360 : diffYaw<-180 ? diffYaw+360 : diffYaw;

        if (degreesToTurn < 0) {
            direction = SpinDirection.COUNTERCLOCKWISE;
        } else {
            direction = SpinDirection.CLOCKWISE;
        }
        return (direction);
    }

    /**
     * Converts the given targetYaw into the angle to turn.  Returns a value between [-180, +180]
     * @param curYaw  current yaw value
     * @param targetYaw  target yaw value
     *      The targetYaw is with respect to the initial autonomous starting position
     *      The initial orientation of the robot at the beginning of the autonomous period
     *      is '0'. targetYaw is between 0 to 360 degrees.

     * @return degreesToTurn
     */
    public double getDegreesToTurn(double curYaw, double targetYaw) {
        double diffYaw = targetYaw - curYaw;
        double degreesToTurn = diffYaw>180 ? diffYaw-360 : diffYaw<-180 ? diffYaw+360 : diffYaw;
        return (degreesToTurn);
    }

    /**
     * Calculates the targetYaw (value between 0 and 359 degrees)
     * @param curYaw
     * @param angleToTurn
     * @return targetYaw
     */
    public double getTargetYaw(double curYaw, double angleToTurn) {
        double sum = curYaw + angleToTurn;
        double targetYaw = (sum>360) ? (sum-360) : ((sum<0) ? (360+sum) : sum);
        return (targetYaw);
    }

    public void untiltRobot(boolean originallyGoingBackward, double degrees, float speed) {
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        // Do not move more than 15 inches; this is the upper limit but not the absolute distance to travel
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(15);
        // Do not move for more than 5 seconds
        NavigationChecks.TimeoutCheck timeoutCheck = navChecks.new TimeoutCheck(5000);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(encodercheck);
        navChecks.addNewCheck(timeoutCheck);
        navChecks.addNewCheck(opmodeCheck);
        // For un-tilting, the robot has to move in the reverse direction of the original direction.
        boolean driveBackwards = originallyGoingBackward ? false : true;

//        robot.instrumentation.addAction(driveToDistInstr);
//        robot.instrumentation.reset();
        // move until the robot tilt goes down below 3 degrees
        while (!navChecks.stopNavigation() && (gyro.getPitch() > 3)) {
            gyro.goStraightPID(driveBackwards, degrees, speed);
//            if (rangeInstr != null)
//                robot.instrumentation.addInstrData();
        }
//        robot.instrumentation.printToConsole();
//        robot.instrumentation.writeToFile();
//        robot.instrumentation.removeAction(driveToDistInstr);
    }

    public void goStraightAlongTheWall(double inches, double degrees, float speed, double targetDistFromWall,
                                       double wallKp) {
        String methodSignature = String.format("goStraightAlongTheWall(inches=%f, degrees=%f, speed=%f," +
                " targetDistFromWall=%f, wallKp=%f)", inches, degrees, speed, targetDistFromWall, wallKp);
        if (robot.printDebugMsg)
            DbgLog.msg("ftc9773: %s", methodSignature);
        // If gyro is working, using gyro's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(inches);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        navChecks.addNewCheck(encodercheck);
        if (rangeInstr != null)
            robot.instrumentation.addAction(rangeInstr);
        boolean driveBackwards = inches < 0 ? true : false;
        double curDistFromWall, angleToMaintain, error, correction;
        if (gyro.isGyroWorking()) {
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Gyro is working");
            NavigationChecks.CheckRobotTilting tiltingCheck = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(tiltingCheck);
            if (gyroDegreesInstr != null)
                robot.instrumentation.addAction(gyroDegreesInstr);
            if (robot.instrLevel != Instrumentation.InstrumentationLevel.NONE)
                robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                curDistFromWall = rangeSensor.getDistance(DistanceUnit.CM);
                error = targetDistFromWall - curDistFromWall;
                correction = wallKp * error;
                if (driveBackwards)
                    correction = -correction;
                angleToMaintain = getTargetYaw(degrees, correction);
                gyro.goStraightPID(driveBackwards, angleToMaintain, speed);
                robot.instrumentation.addInstrData();
                if (tiltingCheck.stopNavigation()) {
                    DbgLog.msg("ftc9773: tilting detected");
                    this.untiltRobot(driveBackwards, degrees, speed);
                }
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            if (gyroDegreesInstr != null)
                robot.instrumentation.removeAction(gyroDegreesInstr);
            if (rangeInstr != null)
                robot.instrumentation.removeAction(rangeInstr);
            // Update the encoderNav's current yaw with that of gyro
            encoderNav.setCurrentYaw(gyro.getYaw());
        }
    }

    public void goStraightToDistance(double inches, double degrees, float speed) {
        String methodSignature = String.format("goStraightToDistance(inches=%f, degrees=%f, speed=%f)",
                inches, degrees, speed);
        if (robot.printDebugMsg)
            DbgLog.msg("ftc9773: %s", methodSignature);
        // If gyro is working, using gyro's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(inches);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        navChecks.addNewCheck(encodercheck);
        if (rangeInstr != null)
            robot.instrumentation.addAction(rangeInstr);
//        robot.instrumentation.addAction(driveToDistInstr);
        boolean driveBackwards = inches < 0 ? true : false;
        if (gyro.isGyroWorking()) {
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Gyro is working");
            NavigationChecks.CheckRobotTilting tiltingCheck = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(tiltingCheck);
            if (gyroDegreesInstr != null)
                robot.instrumentation.addAction(gyroDegreesInstr);
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                gyro.goStraightPID(driveBackwards, degrees, speed);
                robot.instrumentation.addInstrData();
                if (tiltingCheck.stopNavigation()) {
                    DbgLog.msg("ftc9773: tilting detected");
                    this.untiltRobot(driveBackwards, degrees, speed);
                }
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            if (gyroDegreesInstr != null)
                robot.instrumentation.removeAction(gyroDegreesInstr);
            // Update the encoderNav's current yaw with that of gyro
            encoderNav.setCurrentYaw(gyro.getYaw());
        } else {
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Gyro is not working");
            // Use purely encoder based navigation
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Speed: %f, distance: %f", speed, inches);
            if (inches < 0) {
                // If driving backwards, then negate the speed
                speed = -speed;
            }
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(speed, 0);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
        }
        if (rangeInstr != null)
            robot.instrumentation.removeAction(rangeInstr);
//        robot.instrumentation.removeAction(driveToDistInstr);
    }

    public String goStraightToWhiteLine(double degrees, float motorSpeed, boolean driveBackwards,
                                      double distCorrection, String frontOrBackODS) {
        String methodSignature = String.format("goStraightToWhiteLine(degrees=%f, motorSpeed=%f, " +
                "driveBackwards=%b, frontOrBackODS=%s)", degrees, motorSpeed,
                driveBackwards, frontOrBackODS);
        if (robot.printDebugMsg)
            DbgLog.msg("ftc9773: %s", methodSignature);

        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.CheckForWhiteLine odsCheck = navChecks.new CheckForWhiteLine(this.lf, frontOrBackODS);
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(odsCheck);
        navChecks.addNewCheck(check2);
//        robot.instrumentation.addAction(driveTillWhitelineInstr);
        if (odsInstr != null)
            robot.instrumentation.addAction(odsInstr);
        if (colorInstr != null)
            robot.instrumentation.addAction(colorInstr);
        if (gyroDegreesInstr != null)
            robot.instrumentation.addAction(gyroDegreesInstr);
        // Determine the distance from wall and see if beaconServo needs to be retracted
        // It might have been pre-extended before.
        double distFromWall = rangeSensor.getDistance(DistanceUnit.CM);

        if (gyro.isGyroWorking()) {
            NavigationChecks.CheckRobotTilting check3 = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(check3);
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                gyro.goStraightPID(driveBackwards, degrees, motorSpeed);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            if (distCorrection != 0) {
                navChecks.removeCheck(odsCheck);
                NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(distCorrection);
                boolean distCorrctnDrvBack = (distCorrection < 0) ? true : false;
                navChecks.addNewCheck(encodercheck);
                while (!navChecks.stopNavigation()) {
                    gyro.goStraightPID(distCorrctnDrvBack, degrees, motorSpeed);
                    robot.instrumentation.addInstrData();
                }
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            // Update the encoderNav's current yaw with that of gyro
            encoderNav.setCurrentYaw(gyro.getYaw());
        } else {
            // Use purely encoder based navigation
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(motorSpeed, 0);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
//        robot.instrumentation.removeAction(driveTillWhitelineInstr);
        if (gyroDegreesInstr != null)
            robot.instrumentation.removeAction(gyroDegreesInstr);
        if (odsInstr != null)
            robot.instrumentation.removeAction(odsInstr);
        if (colorInstr != null)
            robot.instrumentation.removeAction(colorInstr);
        return (odsCheck.getODSonLine());
    }

    public void driveToAllianceBeaconWhileScanning(double degrees, float motorSpeed,
                                                   boolean driveBackwards, double distCorrection,
                                                   String driveToPosition, String frontOrBackODS) {
        String methodSignature = String.format("driveAndClaimAllianceBeacon(degrees=%f, motorSpeed=%f, driveBackwards=%b," +
                "distCorrection=%f, driveToPosition=%s)",
                degrees, motorSpeed, driveBackwards, distCorrection, driveToPosition);
        //colorInstr.driveToColor(robot.autonomousActions.allianceColor, (float)motorSpeed);
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.CheckForWhiteLine check1 = navChecks.new CheckForWhiteLine(this.lf, frontOrBackODS);
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        NavigationChecks.BeaconColorCheck beaconColorCheck = navChecks.new BeaconColorCheck(colorInstr, robot.autonomousActions.allianceColor);
        navChecks.addNewCheck(check1);
        navChecks.addNewCheck(check2);
        navChecks.addNewCheck(beaconColorCheck);
        robot.instrumentation.addAction(odsInstr);
        robot.instrumentation.addAction(colorInstr);
        robot.instrumentation.addAction(gyroDegreesInstr);

        if (gyro.isGyroWorking()) {
            NavigationChecks.CheckRobotTilting check3 = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(check3);
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                gyro.goStraightPID(driveBackwards, degrees, motorSpeed);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            if (navChecks.stopNavCriterion == check1){
                if (distCorrection != 0) {
                    navChecks.removeCheck(check1);
                    NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(distCorrection);
                    navChecks.addNewCheck(encodercheck);
                    boolean distCorrctnDrvBack = (distCorrection < 0) ? true : false;
                    while (!navChecks.stopNavigation()) {
                        gyro.goStraightPID(distCorrctnDrvBack, degrees, motorSpeed);
                        robot.instrumentation.addInstrData();
                        curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
                    }
                }
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            // Update the encoderNav's current yaw with that of gyro
            encoderNav.setCurrentYaw(gyro.getYaw());

//            colorInstr.driveToColor(robot.autonomousActions.allianceColor, motorSpeed, driveToPosition);
        } else {
            // Use purely encoder based navigation
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
            robot.instrumentation.reset(methodSignature);
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(motorSpeed, 0);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
//        robot.instrumentation.removeAction(driveTillWhitelineInstr);

        DriveSystem.DriveSysPosition driveSysPosition =
                colorInstr.getAllianceColorPosition(robot.autonomousActions.allianceColor);
        robot.instrumentation.removeAction(gyroDegreesInstr);
        robot.instrumentation.removeAction(odsInstr);
        robot.instrumentation.removeAction(colorInstr);
        if (driveSysPosition != null) {
            double distanceInInches = driveSysPosition.getDistanceFromCurPosition();
            distanceInInches = (driveBackwards ? distanceInInches : -distanceInInches);
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: distance to go back from current position = %f", distanceInInches);
            // In red alliance, we have to move at 0 degrees whereas is blue alliance we have to move at 180 degrees.
            double degreesToUse = (robot.autonomousActions.allianceColor.equalsIgnoreCase("red") ? 0 : 180);
            goStraightToDistance(distanceInInches, degreesToUse, motorSpeed);
        }
    }

    public void setRobotOrientation(double targetYaw, double motorSpeed) {
        String methodSignature = String.format("setRobotOrientation(targetYaw=%f, motorSpeed=%f)", targetYaw, motorSpeed);
        // Create a NavigationChecks object to set the terminating conditions
        NavigationChecks navigationChecks = new NavigationChecks(robot, curOpMode, this);
        // Calculate the timeout based on the targetYaw and currentYaw
        // at the rate of 100 milliseconds per degree of rotation at full speed
        double curYaw = encoderNav.getCurrentYaw();
        long timeoutMillis = (long) Math.abs(this.distanceBetweenAngles(targetYaw, curYaw) * 100 / motorSpeed);
        timeoutMillis = 30000;
        NavigationChecks.TimeoutCheck check1 = navigationChecks.new TimeoutCheck(timeoutMillis);
        navigationChecks.addNewCheck(check1);
        NavigationChecks.OpmodeInactiveCheck check2 = navigationChecks.new OpmodeInactiveCheck();
        navigationChecks.addNewCheck(check2);

        if (gyroDegreesInstr != null)
            robot.instrumentation.addAction(gyroDegreesInstr);

        DriveSystem.ElapsedEncoderCounts elapsedEncoderCounts = robot.driveSystem.getNewElapsedCountsObj();
        elapsedEncoderCounts.reset();

        // If the gyro is working, then set the robot orientation with gyro
        if (gyro.isGyroWorking()) {
            curOpMode.telemetry.addData("Set Robot Orientation", "Using Gyro");
            curOpMode.telemetry.update();
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Set Robot Orientation, Using Gyro");
            // The difference between the encoder-based degrees and gyro based degrees can easily
            // go upto 10 degrees even when gyro is working well.  So, we should not have too low
            // value for the CheckWhileTurning constructor.  Ensure that we do not check for less
            // 30 degree deviation between the encoder-based and gyro-based angles.
//            double degreesToCheck = Math.max(this.distanceBetweenAngles(targetYaw, curYaw) /2, 30);
//            NavigationChecks.CrossCheckGyroWhileTurning check3 = navigationChecks.new
//                    CrossCheckGyroWhileTurning(degreesToCheck);
//            navigationChecks.addNewCheck(check3);
            NavigationChecks.CheckGyroIsWorking gyroCheck = navigationChecks.new CheckGyroIsWorking();
            navigationChecks.addNewCheck(gyroCheck);
            NavigationChecks.CheckGyroTargetYawReached targetYawCheck = navigationChecks.new CheckGyroTargetYawReached(targetYaw);
            navigationChecks.addNewCheck(targetYawCheck);
            SpinDirection cwccw = getSpinDirection(gyro.getYaw(), targetYaw);
            double leftPower=0.0, rightPower=0.0;
            if (cwccw == SpinDirection.CLOCKWISE) {
                leftPower = motorSpeed;
            }
            if (cwccw == SpinDirection.COUNTERCLOCKWISE) {
                leftPower = -motorSpeed;
            }
            rightPower = -leftPower;
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
            robot.instrumentation.reset(methodSignature);
            while (!navigationChecks.stopNavigation()) {
                robot.driveSystem.turnOrSpin(leftPower, rightPower);
                robot.instrumentation.addInstrData();
                curOpMode.idle(); // idle() should be called between every 2 iterations in a loop
            }
            this.robot.driveSystem.stop();
            robot.instrumentation.addInstrData();
            robot.instrumentation.printToConsole();
//            gyro.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            if ((navigationChecks.stopNavCriterion != null) &&
                    ((navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CROSSCHECK_GYRO_WITH_ENCODERS)
                    || (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CHECK_GYRO_IS_WORKING)))
            {
                double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                elapsedEncoderCounts.reset();
                curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Gyro");
                curOpMode.telemetry.update();
                if (robot.printDebugMsg)
                    DbgLog.msg("ftc9773: Set Robot Orientation, Not Using Gyro");
//                navigationChecks.removeCheck(check3);
                navigationChecks.removeCheck(gyroCheck);
                encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
                encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                // Set the gyro status again
                gyro.testAndSetGyroStatus();
            } else if (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CHECK_TARGET_YAW_GYRO) {
                // gyro worked without any problems; Set the encoderNav's currentYaw to the gyro yaw value
                encoderNav.setCurrentYaw(gyro.getYaw());
            }
        }
        else {
            // First, do the encoder based turning.
            curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Gyro");
            curOpMode.telemetry.update();
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: Set Robot Orientation, Not Using Gyro");
            encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            encoderNav.updateCurrentYaw(elapsedEncoderCounts.getDegreesTurned());
            if (robot.printDebugMsg)
                DbgLog.msg("ftc9773: currYaw: %f", encoderNav.getCurrentYaw());
        }
        // Add the instrumentation data one last time before removing the instrumentation actions.
        robot.instrumentation.addInstrData();
        if (gyroDegreesInstr != null)
            robot.instrumentation.removeAction(gyroDegreesInstr);
    }

    /**
     *
     * @param shiftDistance +ve => shift right, -ve => shift left
     * @param moveDistance  +ve => move forward; -ve => move backward
     * @param speed
     * @param returnToSamePos
     */
    public void shiftRobot(double shiftDistance, double moveDistance, double speed,
                           boolean returnToSamePos, double startingYaw, double endingYaw){
        boolean isForward = (moveDistance >= 0) ? true : false;
        double diagonal = Math.sqrt(Math.pow(moveDistance, 2) + Math.pow(shiftDistance, 2));
        double angle = 90 - Math.toDegrees(Math.asin(Math.abs(moveDistance/diagonal)));

        // diagonal should have the same sign as the moveDistance
        diagonal *= Math.signum(moveDistance);
        // If moveDistance and shiftDistance have opposite signs -- i.e. move forward & shift left
        //  or move backward & shift right -- then turn counter clockwise, else turn clockwise
        angle *= Math.signum(moveDistance) * Math.signum(shiftDistance);
        if (robot.printDebugMsg)
            DbgLog.msg("ftc9773: shiftDistance=%f, diagonal=%f, moveDistance=%f, isForward=%b, speed=%f, angle=%f",
                shiftDistance, diagonal, moveDistance, isForward, speed, angle);

        // Step 1.  Turn the robot to move forward / backward
        if (startingYaw < 0)
            startingYaw = (gyro.isGyroWorking() ? gyro.getYaw() : encoderNav.getCurrentYaw());
        double turningYaw = this.getTargetYaw(startingYaw, angle);
        if (robot.printDebugMsg)
            DbgLog.msg("ftc9773: startingYaw=%f, turningYaw=%f", startingYaw, turningYaw);
        this.setRobotOrientation(turningYaw, turnMaxSpeed);
        // Step 2:  Drive to diagonal distance
        this.goStraightToDistance(diagonal, turningYaw, (float)speed);
        //Step 3: Turn to the original orientation again
        if (endingYaw >= 0 && endingYaw <= 360)
            this.setRobotOrientation(endingYaw, turnMaxSpeed);
        else
            this.setRobotOrientation(startingYaw, turnMaxSpeed);
        // Step 4:  return to the same position is specified
        if (returnToSamePos) {
            this.goStraightToDistance(-moveDistance, startingYaw, (float)speed);
        }
    }
}
