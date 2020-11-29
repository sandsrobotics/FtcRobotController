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

    ///////////////////
    //other variables//
    ///////////////////
    //robot position
    protected double[] currentRobotPosition = new double[]{startPositionX, startPositionY, startRotation};
    protected int[] lastMotorPos;
    protected int[] currMotorPos;

    //rotation
    volatile Orientation currentAllAxisRotations = new Orientation();
    volatile double currentRotation;
    protected double rotationOffset = -startRotation;

    //angular velocity
    volatile AngularVelocity currentAngularVelocity = new AngularVelocity();

    //other class
    Robot robot;

    Position(Robot robot)
    {
        this.robot = robot;
    }

    //////////
    //angles//
    //////////
    Orientation updateAngles()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angles.thirdAngle *= -1;
        angles.thirdAngle -= rotationOffset;
        angles.thirdAngle = (float)robot.scaleAngle(angles.thirdAngle);
        return angles;
    }

    void resetZAxisRotation() { rotationOffset = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle - startRotation;}

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
        double XMove = (.25 * (-diff[0] + diff[2] + diff[1] - diff[3]))/Movement.ticksPerInchSideways;

        //rotate and add to robot position
        currentRobotPosition[0] += YMove * Math.sin(currentRotation * Math.PI / 180) - XMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[1] += XMove * Math.sin(currentRotation * Math.PI / 180) + YMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[2] = currentRotation;
    }

    void updatePositionFromVuforia(boolean useCurrentPos)
    {
        if(robot.useVuforia)
        {
            if(useCurrentPos)
            {
                if (robot.vision.currentCalculatedRobotLocation != null) {
                    currentRobotPosition[0] = robot.vision.currentCalculatedRobotLocation.getTranslation().get(0);
                    currentRobotPosition[1] = robot.vision.currentCalculatedRobotLocation.getTranslation().get(1);
                }
            }
            else
            {
                if (robot.vision.lastCalculatedRobotLocation != null) {
                    currentRobotPosition[0] = robot.vision.lastCalculatedRobotLocation.getTranslation().get(0);
                    currentRobotPosition[1] = robot.vision.lastCalculatedRobotLocation.getTranslation().get(1);
                }
            }
        }
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
        currentAngularVelocity = robot.imu.getAngularVelocity();
        currentAllAxisRotations = updateAngles();
        currentRotation = currentAllAxisRotations.thirdAngle;
    }

    @Override
    public void run()
    {
        initialize();
        while (!this.isInterrupted() && robot.opMode.opModeIsActive())
        {
            //put run stuff in here
            updateAll();
            if(robot.usePositionTracking) getPosFromEncoder();
        }
    }

    ///////////////
    //stop thread//
    ///////////////
    void stopPosition()
    {
        this.interrupt();
    }
}