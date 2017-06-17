package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
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
    double[] prevPose = new double[3];
    double avgDistance = 0.0;
    DriveSystem.ElapsedEncoderCounts elapsedEncoderCounts;

    public CoordinateSys(FTCRobot robot, LinearOpMode curOpMode, double x, double y, double angle){
        this.robot=robot;
        this.curOpMode=curOpMode;
        this.prevPose[0] = this.pose[0] = x;
        this.prevPose[1] = this.pose[1] = y;
        this.prevPose[2] = this.pose[2] = angle;
        this.elapsedEncoderCounts = robot.driveSystem.getNewElapsedCountsObj();
        this.elapsedEncoderCounts.reset();
    }

    public void updatePose(){
        //TODO: Once all 4 encoder wires are there,remove
        double distance = elapsedEncoderCounts.getDistanceTravelledInInches() * 2;
        double distanceL = elapsedEncoderCounts.getDistanceTravelledInInchesLeft() * 2;
        double distanceR = elapsedEncoderCounts.getDistanceTravelledInInchesRight() * 2;

        pose[2] = prevPose[2] + ((distanceR - distanceL) / robot.distanceBetweenWheels);
        pose[0] = prevPose[0] + distance * Math.cos(prevPose[2]);
        pose[1] = prevPose[1] + distance * Math.sin(prevPose[2]);
        DbgLog.msg("ftc9773: X=%f, Y=%f, angle=%f", pose[0], pose[1], Math.toDegrees(pose[2]));
        DbgLog.msg("ftc9773: dL=%f, dR=%f, D=%f", distanceL, distanceR, distance);

        for (int i=0;i<3;i++){
            prevPose[i]=pose[i];
        }
        elapsedEncoderCounts.reset();
    }

    public double getX(){return pose[0];}
    public double getY(){return pose[1];}
    public double getAngle(){return pose[2];}
}
