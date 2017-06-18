package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.FileRW;

import java.io.File;
import java.security.acl.AclNotFoundException;

/**
 * Created by pranavburugula on 6/17/2017.
 */

public class BoschIMU implements GyroInterface {
    private enum BNO055IMU_status {STATUS_NOT_SET, WORKING, NOT_WORKING}

    BNO055IMU imu;
    Orientation angles;
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigation;
    public double straightPID_kp=0.005, turnPID_kp=0.005;
    double angleTolerance;
    private BNO055IMU_status status= BNO055IMU_status.STATUS_NOT_SET;
    private double updateCount;
    ElapsedTime getYawTimer;
    // Do not bother to call getIntegratedZValue if it has been less than 20 milli seconds
    // since the last time it was called.
    final double GYRO_LATENCY = 20;

    public BoschIMU(FTCRobot robot, LinearOpMode curOpMode, Navigation navigation,
                  double angleTolerance, double straightPID_kp, double turnPID_kp){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigation = navigation;
        this.angleTolerance = angleTolerance;
        this.straightPID_kp = straightPID_kp;
        this.turnPID_kp = turnPID_kp;
        status= BNO055IMU_status.STATUS_NOT_SET;
        updateCount = 0;

        imu = curOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        //initialize
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        DbgLog.msg("ftc9773: Done with initializing BNO055IMU");

        getYawTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        prevYaw = gyro.getHeading();
        getYawTimer.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
    }

    // todo: find out how to return yaw and pitch angles
    // for Navx it is navx_device.getYaw() and navx_device.getPitch()

    @Override
    public void initAfterStart() {return;}

    @Override
    public Navigation.GyroType getGyroType() {
        return (Navigation.GyroType.BNO055IMU);
    }

    @Override
    public void close() {
        imu.close();
    }

    @Override
    public double getYaw() {
        return getModifiedYaw();
    }

    private double getModifiedYaw() {
        double newYaw = 0.0;
        double curYaw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        // Note:  The BNO055IMU outputs values from 0 to 180 degrees and -180 to 0 degrees as the
        // robot spins counter-clockwise. Convert this to a 0 to 360 degrees scale.
        if (curYaw > -180 && curYaw < 0) {
            newYaw = 360 + curYaw;
        } else {
            newYaw = curYaw;
        }
        //Subtract from 360 so all gyro sensors return in same clockwise direction
        // MR gyro return angle on a 0-360 degree scale while spinning clockwise;
        // Navx-micro returns angle 0 to 180 & -180 to 0 scale while spinning clockwise.
        return (360-newYaw);
    }

    @Override
    public double getPitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    @Override
    public double getRoll(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    @Override
    public double getYaw_cc0_180scale() {
        return (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
    }

    @Override
    public boolean isGyroWorking() {
        return true;
    }

    @Override
    public double getUpdateCount() {
        // update count is valid only for navx-micro
        return (updateCount);
    }

    @Override
    public double getAngleTolerance() {
        return (angleTolerance);
    }

    @Override
    public void testAndSetGyroStatus() {
        if (imu.isSystemCalibrated()) {
            DbgLog.msg("ftc9773:  BNO055IMU still calibrating; connection info=%s", imu.getCalibrationStatus());
            status = BNO055IMU_status.NOT_WORKING;
        } else {
            DbgLog.msg("ft9773: BNO055IMU is done with calibration");
            status = BNO055IMU_status.WORKING;
        }
    }

    @Override
    public void goStraightPID(boolean driveBackwards, double degrees, float speed){
        double error = 0.0, correction = 0.0;
        double leftSpeed, rightSpeed;
        error = getYaw() - degrees;
        if (error > 180){
            error -= 360;
        } else if (error <-180){
            error += 360;
        }
        correction = this.straightPID_kp * error / 2;
        // Ensure that 0.25 <= speed <= 0.75 so that correction can be meanigful.
        speed = Range.clip(speed, 0.1f, 0.75f);
        leftSpeed = Range.clip(speed - correction, 0, 1);
        rightSpeed = Range.clip(speed + correction, 0, 1);

        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }
}
