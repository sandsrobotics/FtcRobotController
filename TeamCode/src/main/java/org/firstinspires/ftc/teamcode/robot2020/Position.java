package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Position extends Thread
{



    ///////////////////
    //other variables//
    ///////////////////
    //robot position
    protected volatile double[] currentRobotPosition; //position is based on the canter front of the goal to the center of the robot

    //wheels
    private int[] lastMotorPos;
    private int[] currMotorPos;

    //rotation
    volatile Orientation currentAllAxisRotations = new Orientation();
    volatile double currentRotation;
    protected double rotationOffset;

    //angular velocity
    volatile AngularVelocity currentAngularVelocity = new AngularVelocity();

    //other
    volatile double positionAccuracy = 0;
    int currentRun = 0;

    //other class
    Robot robot;
    PositionSettings positionSettings;

    Position(Robot robot)
    {
        positionSettings = new PositionSettings();
        this.robot = robot;
        initVals();
    }
    Position(Robot robot, PositionSettings positionSettings)
    {
        this.positionSettings = positionSettings;
        this.robot = robot;
        initVals();
    }

    void initVals()
    {
        if(positionSettings.resetPos)
        {
            currentRobotPosition = new double[]{positionSettings.startPositionX, positionSettings.startPositionY, positionSettings.startRotation};
            rotationOffset = -positionSettings.startRotation;
        }
        else
        {
            currentRobotPosition = new double[]{0, 0, 0};
            rotationOffset = 0;
        }
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


    ////////////////////
    //position finding//
    ////////////////////
    void getPosFromEncoder()
    {
        //get difference
        lastMotorPos = currMotorPos;
        currMotorPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
        int[] diff = new int[4];
        for(int i = 0; i < 4; i++)
        {
            diff[i] = currMotorPos[i] - lastMotorPos[i];
        }

        //get movement
        double YMove = (.25 * (diff[0] + diff[2] + diff[1] + diff[3]))/MovementSettings.ticksPerInchForward;
        double XMove = (.25 * (-diff[0] + diff[2] + diff[1] - diff[3]))/MovementSettings.ticksPerInchSideways;

        //rotate and add to robot position
        currentRobotPosition[0] += YMove * Math.sin(currentRotation * Math.PI / 180) - XMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[1] += XMove * Math.sin(currentRotation * Math.PI / 180) + YMove * Math.cos(currentRotation * Math.PI / 180);
        currentRobotPosition[2] = currentRotation;
    }

    ///////////////////
    //distance sensor//
    ///////////////////
    void updatePosWithDistanceSensor()
    {
        for (int i = -1; i < 3; i++)//runs through the 4 90 degree rotation increments
        {
            int arrayPos = i;
            if(i == -1) arrayPos = 3;
            if (Math.abs(currentRotation - (i * 90)) <= positionSettings.angleTolerance) //checks if the current angle is close enough to one of the 90 degree increments
            {
                int[] vals = robot.robotHardware.getDistancesList(robot.robotHardware.distSensors);

                if(positionSettings.sensorPosition[arrayPos] == SensorNum.TWO)
                {
                    int val = vals[0];
                    vals[0] = vals[1];
                    vals[1] = val;
                }

                for (int b = 0; b < 2; b++) //does the math for both x and y axis
                {
                    double dis = positionSettings.distancesFromWall[arrayPos][b];
                    if (positionSettings.operations[arrayPos][b] == MathSign.ADD) { dis += vals[b] * Math.cos(currentRotation); }
                    else { dis -= vals[b] * Math.cos(currentRotation); }
                    currentRobotPosition[b] = dis;
                }
            }
        }
    }

    //////////////////
    //runs in thread//
    //////////////////
    void initialize()
    {
        currMotorPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
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
        if(robot.robotUsage.usePositionTracking) { initialize();}
        while (!this.isInterrupted() && robot.opMode.opModeIsActive())
        {
            //put run stuff in here
            updateAll();
            if(robot.robotUsage.usePositionTracking)
            {
                getPosFromEncoder();
                updatePosWithDistanceSensor();
            }
        }
    }

    double[] getPositionWithOffset(double X, double Y, double R)
    {
        return new double[]{currentRobotPosition[0] + X, currentRobotPosition[1] + Y,currentRobotPosition[2] + R};
    }

    ///////////////
    //stop thread//
    ///////////////
    void stopPosition()
    {
        this.interrupt();
    }
}

@Config
class PositionSettings
{
    //////////////////
    //user variables//
    //////////////////
    //position start
    boolean resetPos = true;
    double startPositionX = -20; // in inches from center of goal
    double startPositionY = -124; // in inches from front of goal
    double startRotation = 0; //in degrees from goal

    //ultra sonic
    int[][] distancesFromWall = new int[][] //these are the distances that the ultra sonic sensors are at while the robot is at the 0 point and at specific angles
    {
        new int[]{0,0}, // for 0 degrees
        new int[]{0,0}, // for 90 degrees
        new int[]{0,0}, // for 180 degrees
        new int[]{0,0}  // for -90/270 degrees
    };
    SensorNum[] sensorPosition = new SensorNum[] // which ultra sonic sensor is in the X direction for each 90 degree increment
    {
        SensorNum.ONE, // for 0 degrees
        SensorNum.ONE, // for 90 degrees
        SensorNum.ONE, // for 180 degrees
        SensorNum.ONE  // for -90/270 degrees
    };
    MathSign[][] operations = new MathSign[][] //whether the distance from the ultrasonic sensor should be added or removed for each 90 degree increment
    {
        new MathSign[]{MathSign.ADD, MathSign.ADD}, // for 0 degrees
        new MathSign[]{MathSign.ADD, MathSign.ADD}, // for 90 degrees
        new MathSign[]{MathSign.ADD, MathSign.ADD}, // for 180 degrees
        new MathSign[]{MathSign.ADD, MathSign.ADD}  // for -90/270 degrees
    };
    double angleTolerance = 7.5; // how far from each 90 degree increment can the robot be for the ultra sonic to still be valid

    PositionSettings(){}
}

enum MathSign
{
    ADD,
    SUBTRACT
}
enum SensorNum
{
    ONE,
    TWO
}