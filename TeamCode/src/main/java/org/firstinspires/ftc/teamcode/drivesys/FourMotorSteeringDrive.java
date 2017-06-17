/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.NavigationChecks;

public class FourMotorSteeringDrive extends DriveSystem {
    DcMotor motorL1 = null;
    DcMotor motorL2 = null;
    DcMotor motorR1 = null;
    DcMotor motorR2 = null;
    double prevPowerL1, prevPowerL2, prevPowerR1, prevPowerR2;
    double frictionCoefficient;
    int maxSpeedCPS; // encoder counts per second
    public Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40, 560 for Neverest20
    boolean driveSysIsReversed = false;
    double distBetweenWheels;
    boolean L1IsZero, L2IsZero, R1IsZero, R2IsZero;
    ElapsedTime L1Timer, L2Timer, R1Timer, R2Timer;
    double scaleDriveMultiplier = 1.0;



    double scaleSpinMultiplier=1.0;
    int reverseMultiplier = 1;

    public class ElapsedEncoderCounts implements DriveSystem.ElapsedEncoderCounts {
        long encoderCountL1;
        long encoderCountL2;
        long encoderCountR1;
        long encoderCountR2;

        public ElapsedEncoderCounts() {
            encoderCountL1 = encoderCountL2 = encoderCountR1 = encoderCountR2 = 0;
        }

        public void reset() {
            encoderCountL1 = getNonZeroCurrentPos(motorL1);
            encoderCountL2 = getNonZeroCurrentPos(motorL2);
            encoderCountR1 = getNonZeroCurrentPos(motorR1);
            encoderCountR2 = getNonZeroCurrentPos(motorR2);
//            DbgLog.msg("ftc9773: In reset(): encoder counts: L1=%d, L2=%d, R1=%d, R2=%d", encoderCountL1,
//                    encoderCountL2, encoderCountR1, encoderCountR2);
        }

        public  void printCurrentEncoderCounts() {
            DbgLog.msg("ftc9773: In printCurrent...(): encoder counts: L1=%d, L2=%d, R1=%d, R2=%d",
                    motorL1.getCurrentPosition(), getNonZeroCurrentPos(motorL2),
                    getNonZeroCurrentPos(motorR1), getNonZeroCurrentPos(motorR2));
        }

        public double getDistanceTravelledInInches() {
            double avgEncoderCounts = 0.0;
            double distanceTravelled = 0.0;

            avgEncoderCounts = ((motorL1.getCurrentPosition() - encoderCountL1) +
                    (motorL2.getCurrentPosition() - encoderCountL2) +
                    (motorR1.getCurrentPosition() - encoderCountR1) +
                    (motorR2.getCurrentPosition() - encoderCountR2)) / 4;

            distanceTravelled = (avgEncoderCounts / motorCPR) * wheel.getCircumference();
            return (distanceTravelled);
        }

        public double getDistanceTravelledInInchesLeft(){
            double avgEncoderCounts = 0.0;
            double distanceTravelled = 0.0;

            avgEncoderCounts = ((motorL1.getCurrentPosition() - encoderCountL1) +
                    (motorL2.getCurrentPosition() - encoderCountL2)) / 2;

            distanceTravelled = (avgEncoderCounts / motorCPR) * wheel.getCircumference();
            return (distanceTravelled);
        }

        public double getDistanceTravelledInInchesRight(){
            double avgEncoderCounts = 0.0;
            double distanceTravelled = 0.0;

            avgEncoderCounts = ((motorR1.getCurrentPosition() - encoderCountR1) +
                    (motorR2.getCurrentPosition() - encoderCountR2)) / 2;

            distanceTravelled = (avgEncoderCounts / motorCPR) * wheel.getCircumference();
            return (distanceTravelled);
        }

        public double getDegreesTurned() {
            double distanceTravelledInInches, degreesTurned;
            double leftDegreesTurned;

            distanceTravelledInInches = this.getDistanceTravelledInInches();
            degreesTurned = 360 * distanceTravelledInInches / (Math.PI * distBetweenWheels);
            leftDegreesTurned = ((getNonZeroCurrentPos(motorL1) - encoderCountL1) +
                    (getNonZeroCurrentPos(motorL2) - encoderCountL2)) / 2;
            if (leftDegreesTurned < 0) {
                degreesTurned *= -1; // Negate the number to indicate counterclockwise spin
            }
//            DbgLog.msg("ftc9773: distanceTravelledInInches: %f, degreesTurned: %f", distanceTravelledInInches, degreesTurned);

            return (degreesTurned);
        }

        @Override
        public void copyFrom(DriveSystem.ElapsedEncoderCounts otherCounts) {
            ElapsedEncoderCounts otherElapsedCounts = (ElapsedEncoderCounts) otherCounts;
            this.encoderCountL1 = otherElapsedCounts.encoderCountL1;
            this.encoderCountL2 = otherElapsedCounts.encoderCountL2;
            this.encoderCountR1 = otherElapsedCounts.encoderCountR1;
            this.encoderCountR2 = otherElapsedCounts.encoderCountR2;
        }
    }

    public class DriveSysPosition implements DriveSystem.DriveSysPosition {
        long encoderCountL1;
        long encoderCountL2;
        long encoderCountR1;
        long encoderCountR2;

        public DriveSysPosition() {
            encoderCountL1 = encoderCountL2 = encoderCountR1 = encoderCountR2 = 0;
        }

        @Override
        public void savePostion() {
            encoderCountL1 = getNonZeroCurrentPos(motorL1);
            encoderCountL2 = getNonZeroCurrentPos(motorL2);
            encoderCountR1 = getNonZeroCurrentPos(motorR1);
            encoderCountR2 = getNonZeroCurrentPos(motorR2);
        }

        @Override
        public void resetPosition() {
            encoderCountL1 = encoderCountL2 = encoderCountR1 = encoderCountR2 = 0;
        }

        @Override
        public void driveToPosition(double speed) {
            DbgLog.msg("ftc9773: drive to a previously saved position");

            motorL1.setTargetPosition((int)encoderCountL1);
            motorL2.setTargetPosition((int)encoderCountL2);
            motorR1.setTargetPosition((int)encoderCountR1);
            motorR2.setTargetPosition((int)encoderCountR2);
            setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive((float) (speed * frictionCoefficient), 0.0f);
            while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy()
                    && curOpMode.opModeIsActive()) {
                curOpMode.idle();
            }
            stop();
            setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        @Override
        public void driveToMidPosition(DriveSystem.DriveSysPosition driveSysPosition, double speed){
            DriveSysPosition driveSysPosition1 = (DriveSysPosition)driveSysPosition;
            long midL1 = (this.encoderCountL1+driveSysPosition1.encoderCountL1)/2;
            long midL2 = (this.encoderCountL2+driveSysPosition1.encoderCountL2)/2;
            long midR1 = (this.encoderCountR1+driveSysPosition1.encoderCountR1)/2;
            long midR2 = (this.encoderCountR2+driveSysPosition1.encoderCountR2)/2;

            DbgLog.msg("ftc9773: position1: L1=%d, L2=%d, R1=%d, R2=%d", this.encoderCountL1,
                    this.encoderCountL2, this.encoderCountR1, this.encoderCountR2);
            DbgLog.msg("ftc9773: position2: L1=%d, L2=%d, R1=%d, R2=%d",
                    driveSysPosition1.encoderCountL1, driveSysPosition1.encoderCountL2,
                    driveSysPosition1.encoderCountR1, driveSysPosition1.encoderCountR2);
            DbgLog.msg("ftc9773: mid position: L1=%d, L2=%d, R1=%d, R2=%d", midL1, midL2,
                    midR1, midR2);

            motorL1.setTargetPosition((int)midL1);
            motorL2.setTargetPosition((int)midL2);
            motorR1.setTargetPosition((int)midR1);
            motorR2.setTargetPosition((int)midR2);
            setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive((float) (speed * frictionCoefficient), 0.0f);
            while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy()
                    && curOpMode.opModeIsActive()) {
                curOpMode.idle();
            }
            stop();
            setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        @Override
        public double getDistanceFromCurPosition() {
            double avgEncoderCounts = 0.0;
            double distanceTravelled = 0.0;

            avgEncoderCounts = (Math.abs(motorL1.getCurrentPosition() - encoderCountL1) +
                    Math.abs(getNonZeroCurrentPos(motorL2) - encoderCountL2) +
                    Math.abs(getNonZeroCurrentPos(motorR1) - encoderCountR1) +
                    Math.abs(getNonZeroCurrentPos(motorR2) - encoderCountR2)) / 4;

            distanceTravelled = (avgEncoderCounts / motorCPR) * wheel.getCircumference();
            return (distanceTravelled);
        }
    }

    public FourMotorSteeringDrive(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1, DcMotor motorR2,
                                  int maxSpeedCPS, double frictionCoefficient,
                                  double distanceBetweenWheels, Wheel wheel, int motorCPR,
                                  String autoOrTeleop) {
        this.motorL1 = motorL1;
        this.motorL2 = motorL2;
        this.motorR1 = motorR1;
        this.motorR2 = motorR2;
        this.motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorL2.setDirection(DcMotorSimple.Direction.FORWARD);
        if (autoOrTeleop.equalsIgnoreCase("Autonomous")) {
            this.setDriveSysMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset the encoders first
            this.setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            this.setDriveSysMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frictionCoefficient = frictionCoefficient;
        this.wheel = wheel;
        this.motorCPR = motorCPR;
        this.prevPowerL1 = this.prevPowerL2 = this.prevPowerR1 = this.prevPowerR2 = 0.0;
        this.distBetweenWheels = distanceBetweenWheels; // 14.75 or 15.5;
        this.L1IsZero = this.L2IsZero = this.R1IsZero = this.R2IsZero = true;
        this.L1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.L2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.R1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.R2Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        L1Timer.reset();
        L2Timer.reset();
        R1Timer.reset();
        R2Timer.reset();
    }


    @Override
    public void drive(float speed, float direction) {
        double left = 0.0, right = 0.0;
        if (speed != 0.0) {
            left = ((reverseMultiplier * speed + direction) * frictionCoefficient) * scaleDriveMultiplier;
            right = ((reverseMultiplier * speed - direction) * frictionCoefficient) * scaleDriveMultiplier;
        } else if (speed == 0.0){
            left = ((reverseMultiplier * speed + direction) * frictionCoefficient) * scaleSpinMultiplier;
            right = ((reverseMultiplier * speed - direction) * frictionCoefficient) * scaleSpinMultiplier;
        }
//        motorL1.setPower(left);
//        motorL2.setPower(left);
//        motorR1.setPower(right);
//        motorR2.setPower(right);

        if (prevPowerL1 != left) {
            motorL1.setPower(left);
            prevPowerL1 = left;
        }
        if (prevPowerL2 != left) {
            motorL2.setPower(left);
            prevPowerL2 = left;
        }
        if (prevPowerR1 != right) {
            motorR1.setPower(right);
            prevPowerR1 = right;
        }

        if (prevPowerR2 != right) {
            motorR2.setPower(right);
            prevPowerR2 = right;
        }
    }

    @Override
    public void turnOrSpin(double left, double right) {
//        motorL1.setPower(left);
//        motorL2.setPower(left);
//        motorR1.setPower(right);
//        motorR2.setPower(right);

        if (prevPowerL1 != left) {
            motorL1.setPower(left);
            prevPowerL1 = left;
        }
        if (prevPowerL2 != left) {
            motorL2.setPower(left);
            prevPowerL2 = left;
        }
        if (prevPowerR1 != right) {
            motorR1.setPower(right);
            prevPowerR1 = right;
        }

        if (prevPowerR2 != right) {
            motorR2.setPower(right);
            prevPowerR2 = right;
        }
    }

    @Override
    public void stop() {
        motorL1.setPower(0.0);
        motorL2.setPower(0.0);
        motorR1.setPower(0.0);
        motorR2.setPower(0.0);
        prevPowerL1 = prevPowerL2 = prevPowerR1 = prevPowerR2 = 0.0;
    }

    @Override
    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior zp_behavior) {
        this.motorL1.setZeroPowerBehavior(zp_behavior);
        this.motorL2.setZeroPowerBehavior(zp_behavior);
        this.motorR1.setZeroPowerBehavior(zp_behavior);
        this.motorR2.setZeroPowerBehavior(zp_behavior);
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return (motorL1.getZeroPowerBehavior());
    }

    @Override
    public void driveToDistance(float speed, double distanceInInches) {

        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distanceInInches;

        motorL1.setTargetPosition(getNonZeroCurrentPos(motorL1) + (int) targetCounts);
        motorL2.setTargetPosition(getNonZeroCurrentPos(motorL2) + (int) targetCounts);
        motorR1.setTargetPosition(getNonZeroCurrentPos(motorR1) + (int) targetCounts);
        motorR2.setTargetPosition(getNonZeroCurrentPos(motorR2) + (int) targetCounts);

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.drive((float) (speed * frictionCoefficient), 0.0f);

        while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy() && curOpMode.opModeIsActive()) {
            curOpMode.idle();
        }

        this.stop();

        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void turnDegrees(double degrees, float speed, NavigationChecks navExc) {

        double distInInches = (Math.abs(degrees) / 360) * Math.PI * this.distBetweenWheels;
        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distInInches;
        double L1targetCounts, L2targetCounts, R1targetCounts, R2targetCounts;

        if (degrees < 0) {
            // Spin counterclockwise => left motors backward, right motors forward
            motorL1.setTargetPosition(getNonZeroCurrentPos(motorL1) - (int) targetCounts);
            motorL2.setTargetPosition(getNonZeroCurrentPos(motorL2) - (int) targetCounts);
            motorR1.setTargetPosition(getNonZeroCurrentPos(motorR1) + (int) targetCounts);
            motorR2.setTargetPosition(getNonZeroCurrentPos(motorR2) + (int) targetCounts);
        } else {
            // Spin clockwise => left motors forward, right motors backward
            motorL1.setTargetPosition(getNonZeroCurrentPos(motorL1) + (int) targetCounts);
            motorL2.setTargetPosition(getNonZeroCurrentPos(motorL2) + (int) targetCounts);
            motorR1.setTargetPosition(getNonZeroCurrentPos(motorR1) - (int) targetCounts);
            motorR2.setTargetPosition(getNonZeroCurrentPos(motorR2) - (int) targetCounts);
        }
        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL1));

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.drive((float) (speed * frictionCoefficient), 0.0f);

        while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy()
                && curOpMode.opModeIsActive()) {
//                && !navExc.stopNavigation() && curOpMode.opModeIsActive()) {
            curOpMode.idle();
        }

        this.stop();

        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL1));
        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveToEncoderCounts(DriveSystem.ElapsedEncoderCounts fromC,
                                     DriveSystem.ElapsedEncoderCounts toC,
                                     float speed) {
        int targetCountsL1, targetCountsL2, targetCountsR1, targetCountsR2;
        ElapsedEncoderCounts fromCounts = (ElapsedEncoderCounts) fromC;
        ElapsedEncoderCounts toCounts = (ElapsedEncoderCounts) toC;

        targetCountsL1 = (int)(toCounts.encoderCountL1 - fromCounts.encoderCountL1);
        targetCountsL2 = (int)(toCounts.encoderCountL2 - fromCounts.encoderCountL2);
        targetCountsR1 = (int)(toCounts.encoderCountR1 - fromCounts.encoderCountR1);
        targetCountsR2 = (int)(toCounts.encoderCountR2 - fromCounts.encoderCountR2);

        motorL1.setTargetPosition(getNonZeroCurrentPos(motorL1) + targetCountsL1);
        motorL2.setTargetPosition(getNonZeroCurrentPos(motorL2) +  targetCountsL2);
        motorR1.setTargetPosition(getNonZeroCurrentPos(motorR1) +  targetCountsR1);
        motorR2.setTargetPosition(getNonZeroCurrentPos(motorR2) +  targetCountsR2);
        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL1));

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.drive((float) (speed * frictionCoefficient), 0.0f);

        while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy()
                && curOpMode.opModeIsActive()) {
//                && !navExc.stopNavigation() && curOpMode.opModeIsActive()) {
            curOpMode.idle();
        }

        this.stop();

        DbgLog.msg("ftc9773: motorL1 current position = %d", getNonZeroCurrentPos(motorL1));
        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setDriveSysMode(DcMotor.RunMode runMode) {
        motorL1.setMode(runMode);
        motorL2.setMode(runMode);
        motorR1.setMode(runMode);
        motorR2.setMode(runMode);
    }

    @Override
    public void reverse() {
        if (driveSysIsReversed) {
            motorL1.setDirection(DcMotorSimple.Direction.REVERSE);
            motorL2.setDirection(DcMotorSimple.Direction.REVERSE);
            motorR1.setDirection(DcMotorSimple.Direction.FORWARD);
            motorR2.setDirection(DcMotorSimple.Direction.FORWARD);
            driveSysIsReversed = false;
        } else {
            motorL1.setDirection(DcMotorSimple.Direction.FORWARD);
            motorL2.setDirection(DcMotorSimple.Direction.FORWARD);
            motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
            motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
            driveSysIsReversed = true;
        }
    }

    public ElapsedEncoderCounts getNewElapsedCountsObj() {
        ElapsedEncoderCounts encoderCountsObj = new ElapsedEncoderCounts();
        return (encoderCountsObj);
    }

    @Override
    public DriveSystem.DriveSysPosition getNewDrivesysPositionObj() {
        DriveSysPosition driveSysPositionObj = new DriveSysPosition();
        return (driveSysPositionObj);
    }

    @Override
    public void printCurrentPosition() {
        DbgLog.msg("ftc9773: L1:%d, L2:%d, R1:%d, R2:%d", getNonZeroCurrentPos(motorL1),
                getNonZeroCurrentPos(motorL2), getNonZeroCurrentPos(motorR1), getNonZeroCurrentPos(motorR2));
    }

    @Override
    public void initForPlay() {
        L1Timer.reset();
        L2Timer.reset();
        R1Timer.reset();
        R2Timer.reset();
    }

    @Override
    public String getDriveSysInstrData() {
        String instrData = String.format("%f,%f,%f,%f,%d,%d,%d,%d", prevPowerL1, prevPowerL2,
                prevPowerR1, prevPowerR2, motorL1.getCurrentPosition(), motorL2.getCurrentPosition(),
                motorR1.getCurrentPosition(), motorR2.getCurrentPosition());

        return instrData;
    }

    public int getNonZeroCurrentPos(DcMotor motor){
        int curPos = motor.getCurrentPosition();
        if (true) {
            return curPos;
        }
        //TODO: Temporary fix to avoid teleop problems with rev
        boolean skipWhileLoop = false;
//        DbgLog.msg("ftc9773: Motor = %s, curPos = %d, isZeroPos = %b", motor.toString(), curPos, motor==motorL1 ? L1IsZero : motor==motorR1 ? R1IsZero : motor==motorL2 ? L2IsZero : R2IsZero);
        if(motor==motorL1 && L1IsZero) {
//            DbgLog.msg("ftc9773: Motor = L1, curPos = %d, isZeroPos = %b", curPos, L1IsZero);
            if(L1Timer.milliseconds()>100) L1IsZero = false;
            skipWhileLoop = true;}
        if(motor==motorL2 && L2IsZero) {
//            DbgLog.msg("ftc9773: Motor = L2, curPos = %d, isZeroPos = %b", curPos, L2IsZero);
            if(L2Timer.milliseconds()>100) L2IsZero = false;
            skipWhileLoop = true;
        }
        if(motor==motorR1 && R1IsZero) {
//            DbgLog.msg("ftc9773: Motor = R1, curPos = %d, isZeroPos = %b",  curPos, R1IsZero);
            if(R1Timer.milliseconds()>100) R1IsZero = false;
            skipWhileLoop = true;
        }
        if(motor==motorR2 && R2IsZero) {
//            DbgLog.msg("ftc9773: Motor = R2, curPos = %d, isZeroPos = %b", curPos, R2IsZero);
            if(R2Timer.milliseconds()>100) R2IsZero = false;
            skipWhileLoop = true;
        }
        if (skipWhileLoop) return curPos;

       while(curPos==0){
           curOpMode.sleep(10);
           curPos = motor.getCurrentPosition();
        }
//        DbgLog.msg("ftc9773: Motor = %s, curPos = %d, isZeroPos = %b", motor.toString(), curPos, motor==motorL1 ? L1IsZero : motor==motorR1 ? R1IsZero : motor==motorL2 ? L2IsZero : R2IsZero);
        return curPos;
    }

    /*public void driveToDistance(float speed, float direction, double distance){
        double startingPositionL = getNonZeroCurrentPos(motorL1);
        double startingPositionR = getNonZeroCurrentPos(motorR1);

        double targetPosition =(distance / wheelValues[1]) * 1120;

        while(((getNonZeroCurrentPos(motorL1)-startingPositionL)<targetPosition) && ((motorR.getCurrentPosition()-startingPositionR)<targetPosition)){
            drive(speed, direction);
        }
        motorR.setPower(0);
        motorL.setPower(0);
    }*/

    public boolean motorControllerIsConnected() {
        boolean connected = false;

        return (connected);
    }

    @Override
    public void scaleDrivePower(double scaleDriveMultiplier){
        this.scaleDriveMultiplier = scaleDriveMultiplier;
    }
    @Override
    public double getScaleDriveMultiplier(){ return scaleDriveMultiplier;}

    @Override
    public void scaleSpinPower(double scaleSpinMultiplier){
        this.scaleSpinMultiplier = scaleSpinMultiplier;
    }

    @Override
    public double getScaleSpinMultiplier() {
        return scaleSpinMultiplier;
    }

    @Override
    public void reverseTeleop(){
        reverseMultiplier = -1;
    }

    @Override
    public void unreverseTeleop() { reverseMultiplier = 1;}

    @Override
    public double[] getCurPosition(){
        long[] curRawPosition = new long[4];
        curRawPosition[0] = motorL1.getCurrentPosition();
        curRawPosition[1] = motorL2.getCurrentPosition();
        curRawPosition[2] = motorR1.getCurrentPosition();
        curRawPosition[3] = motorR2.getCurrentPosition();

        long[] curAvgRawPosition = new long[2];
        curAvgRawPosition[0] = (curRawPosition[0] + curRawPosition[1])/2;
        curAvgRawPosition[1] = (curRawPosition[2] + curRawPosition[3])/2;

        double[] curInchPosition = new double[2];
        curInchPosition[0] = curAvgRawPosition[0] / (motorCPR / wheel.getCircumference());
        curInchPosition[1] = curAvgRawPosition[1] / (motorCPR / wheel.getCircumference());

        return curInchPosition;
    }
}
