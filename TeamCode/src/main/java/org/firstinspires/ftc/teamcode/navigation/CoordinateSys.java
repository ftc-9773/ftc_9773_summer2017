package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.drivesys.FourMotorSteeringDrive;

/**
 * Created by pranavburugula on 6/13/2017.
 */

public class CoordinateSys {
    FTCRobot robot=null;
    LinearOpMode curOpMode=null;
    double[] pose = new double[3];
    double avgDistance = 0.0;

    public CoordinateSys(FTCRobot robot, LinearOpMode curOpMode, double x, double y, double angle){
        this.robot=robot;
        this.curOpMode=curOpMode;
        this.pose[0] = x;
        this.pose[1] = y;
        this.pose[2] = angle;
    }

    public void updatePose(){
        double[] curPosition = robot.driveSystem.getCurPosition();
        avgDistance = (curPosition[0] + curPosition[1]) / 2;

        pose[2] += (curPosition[0] + curPosition[1]) / robot.distanceBetweenWheels;
        pose[0] += avgDistance * Math.cos(pose[2]);
        pose[1] += avgDistance * Math.sin(pose[2]);
    }

    public double getX(){return pose[0];}
    public double getY(){return pose[1];}
    public double getAngle(){return pose[2];}
}
