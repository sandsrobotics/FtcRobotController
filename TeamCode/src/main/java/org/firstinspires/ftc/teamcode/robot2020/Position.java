package org.firstinspires.ftc.teamcode.robot2020;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Position extends Thread
{
    //////////////////
    //user variables//
    //////////////////
    //position start
    double startPositionX = 0; // in inches
    double startPositionY = 0; // in inches
    double startRotation = 0; //in degrees from goal

    //for using encoders
    public static double wheelDistanceFromCenter = 9.6; //in inches

    ///////////////////
    //other variables//
    ///////////////////
    //robot position
    protected double[] currentRobotPosition = new double[]{startPositionX, startPositionY, startRotation};
    protected int[] lastMotorPos;
    protected int[] currMotorPos;

    double a = Math.atan(Movement.ticksPerInchForward/Movement.ticksPerInchSideways);

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
        Velocity out = currentVelocity;//robot.imu.getVelocity();

        out.xVeloc -= velocityOffset.xVeloc;
        out.yVeloc -= velocityOffset.yVeloc;
        out.zVeloc -= velocityOffset.zVeloc;
        return out;

        //return currentVelocity;
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
    void getPosFromEncoder()
    {
        //get difference
        lastMotorPos = currMotorPos;
        currMotorPos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
        int[] diff = new int[4];
        for(int i = 0; i < 4; i++)
        {
            diff[i] = currMotorPos[i] - lastMotorPos[i];
        }

        //get movement
        double YMove = (.25 * (diff[0] + diff[2] + diff[1] + diff[3]))/Movement.ticksPerInchForward;
        double XMove = (.5 * (-diff[0] + diff[2] + diff[1] - diff[3]))/Movement.ticksPerInchSideways;

        //rotate and add to robot position
        currentRobotPosition[0] += YMove * Math.sin(currentRotation * Math.PI / 180) - XMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[1] += XMove * Math.sin(currentRotation * Math.PI / 180) + YMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[2] = currentRotation;
    }

    //////////////////
    //runs in thread//
    //////////////////
    void initialize()
    {
        currMotorPos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
    }

    void updateAll()
    {
        currentAcceleration = updateAcceleration();
        currentVelocity = updateVelocity();
        currentAngularVelocity = robot.imu.getAngularVelocity();
        currentAllAxisRotations = updateAngles();
        currentRotation = currentAllAxisRotations.thirdAngle;
        getPosFromEncoder();
    }

    @Override
    public void run()
    {
        initialize();
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
