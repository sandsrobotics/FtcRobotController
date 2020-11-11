package org.firstinspires.ftc.teamcode.robot2020;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.List;

public class Position extends Thread
{
    //////////////////
    //user variables//
    //////////////////
    //position start
    double startPositionX = 5; // in inches
    double startPositionY = 5; // in inches
    double startRotation = 5; //in degrees from goal

    //for using encoders
    public static double wheelDistanceFromCenter = 9.6; //in inches

    ///////////////////
    //other variables//
    ///////////////////
    //robot position
    protected double[] currentPosition = new double[]{startPositionX, startPositionY, startRotation};
    double[] recordedAccelAndTime;
    double[] lastRecodedAccelAndTime;

    //rotation
    volatile Orientation currentAllAxisRotations = new Orientation();
    volatile double currentRotation;
    protected double rotationOffset = -startRotation;

    //velocity
    volatile Velocity currentVelocity = new Velocity();
    volatile Velocity velocityOffset = new Velocity();

    //angular velocity
    volatile AngularVelocity currentAngularVelocity = new AngularVelocity();

    //acceleration
    volatile Acceleration currentAcceleration = new Acceleration();
    volatile Acceleration accelerationOffset = new Acceleration();

    //other class
    Robot robot;

    Position(Robot robot){this.robot = robot;}

    //////////
    //angles//
    //////////
    Orientation updateAngles()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angles.thirdAngle *= -1;
        angles.thirdAngle -= rotationOffset;
        if(angles.thirdAngle < -180) {angles.thirdAngle = 360 + angles.thirdAngle;}
        else if(angles.thirdAngle > 180){angles.thirdAngle = angles.thirdAngle - 360;}
        return angles;
    }

    void resetZAxisRotation() { rotationOffset = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle - startRotation;}

    ////////////
    //velocity//
    ////////////
    void resetVelocity()
    {
        velocityOffset = robot.imu.getVelocity();
    }

    Velocity updateVelocity()
    {

        Velocity out = robot.imu.getVelocity();

        if(out.xVeloc > 1 && out.yVeloc > 5 && out.zVeloc > 10)
        {
            out.xVeloc -= velocityOffset.xVeloc;
            out.yVeloc -= velocityOffset.yVeloc;
            out.zVeloc -= velocityOffset.zVeloc;
            return out;
        }
        return currentVelocity;
    }

    ////////////////
    //acceleration//
    ////////////////
    void resetAcceleration()
    {
        accelerationOffset = robot.imu.getAcceleration();
    }

    Acceleration updateAcceleration()
    {
        Acceleration out = robot.imu.getAcceleration();
        out.xAccel -= accelerationOffset.xAccel;
        out.yAccel -= accelerationOffset.yAccel;
        out.zAccel -= accelerationOffset.zAccel;
        return out;
    }

    ////////////////////
    //position finding//
    ////////////////////
    void updateMovementFromAccelerometer()
    {
        double[] moved = new double[3];

        currentPosition[2] = currentRotation;
    }

    void addAccelAndTimeDataPoint()
    {
        double[] data = new double[4];
        data[0] = currentAcceleration.xAccel;
        //double
    }

    //////////////////
    //runs in thread//
    //////////////////
    void updateAll()
    {
        currentAcceleration = updateAcceleration();
        currentVelocity = updateVelocity();
        currentAngularVelocity = robot.imu.getAngularVelocity();
        currentAllAxisRotations = updateAngles();
        currentRotation = currentAllAxisRotations.thirdAngle;
    }

    @Override
    public void run()
    {
        while (!Thread.currentThread().isInterrupted() && robot.opMode.opModeIsActive())
        {
            //put run stuff in here
            updateAll();

        }
    }

    ///////////////
    //stop thread//
    ///////////////
    void stopPosition()
    {
        Thread.currentThread().interrupt();
    }
}
