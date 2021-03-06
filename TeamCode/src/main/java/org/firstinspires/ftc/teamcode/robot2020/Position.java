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
    protected volatile double[] currentRobotPosition = new double[3]; //position is based on the canter front of the goal to the center of the robot

    //wheels
    private int[] lastMotorPos;
    private int[] currMotorPos;

    //rotation
    volatile Orientation currentAllAxisRotations = new Orientation();
    volatile double currentRotation;
    protected double rotationOffset;

    //angular velocity
    volatile AngularVelocity currentAngularVelocity = new AngularVelocity();

    //distance sensor position
    float[] lastDistances;
    float[] currentDistances;
    long lastSensorReadingTime = System.currentTimeMillis();
    int inMeasuringRange = -2;

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

    void resetAngle()
    {
        rotationOffset += currentRotation;
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

    int isRobotInRotationRange()//checks if the current angle is close enough to one of the 90 degree increments
    {
        for(int i = -1; i < 3; i++) if(Math.abs(currentRotation - (i * 90)) <= positionSettings.angleTolerance) return i;
        return -2;
    }

    private double distanceFromClosestIncrement() { return Math.abs(currentRotation - (inMeasuringRange * 90)); }

    private void updatePosWithDistanceSensor(boolean useCorrection)
    {
        int arrayPos = inMeasuringRange;
        if(inMeasuringRange == -1) arrayPos = 3;

        float[] temp = robot.robotHardware.getDistancesAfterMeasure(robot.robotHardware.distSensors);
        lastDistances = currentDistances;

        if(useCorrection) {
            for (int i = 0; i < temp.length; i++) {
                if (Math.abs(temp[i] - lastDistances[i]) > positionSettings.maxDistanceSensorChange[i])
                    return;
            }
        }
        lastSensorReadingTime = System.currentTimeMillis();
        currentDistances = temp;

        if(positionSettings.sensorPosition[arrayPos] == SensorNum.TWO)
        {
            float val = temp[0];
            temp[0] = temp[1];
            temp[1] = val;
        }

        for (int b = 0; b < 2; b++) //does the math for both x and y axis
        {
            double dis = positionSettings.distancesFromWall[arrayPos][b];
            if (positionSettings.operations[arrayPos][b] == MathSign.ADD) dis += temp[b] * Math.cos(Math.toRadians(distanceFromClosestIncrement()));
            else dis -= temp[b] * Math.cos(Math.toRadians(distanceFromClosestIncrement()));
            currentRobotPosition[b] = dis;
        }
    }

    //////////////////
    //runs in thread//
    //////////////////
    void initialize()
    {
        currMotorPos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.driveMotors);
        if(robot.robotUsage.useDistanceSensors) currentDistances = robot.robotHardware.getDistancesList(robot.robotHardware.distSensors);
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
        while (!this.isInterrupted() && !robot.opMode.isStopRequested())
        {
            //put run stuff in here
            if(System.currentTimeMillis() - lastSensorReadingTime >= positionSettings.minMeasureDelay) inMeasuringRange = isRobotInRotationRange();
            else inMeasuringRange = -2;
            if(robot.robotUsage.usePositionTracking && robot.robotUsage.useDistanceSensors && inMeasuringRange > -2)
            {
                robot.robotHardware.setSensorsToMeasure(robot.robotHardware.distSensors);
            }
            updateAll();
            if(robot.robotUsage.usePositionTracking)
            {
                getPosFromEncoder();
                if(robot.robotUsage.useDistanceSensors && inMeasuringRange > -2) updatePosWithDistanceSensor(true);
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
    boolean resetPos = true; //if you want to reset the position or set all values to 0
    double startPositionX = -20; // in inches from center of goal
    double startPositionY = -124; // in inches from front of goal
    double startRotation = 0; //in degrees from goal

    //ultra sonic
    float[][] distancesFromWall = new float[][] //these are the distances that the ultra sonic sensors are at while the robot is at the 0 point and at specific angles
    {
        new float[]{-28.740158f, 1.968504f}, // for 0 degrees
        new float[]{52.362202f, 1.968504f}, // for 90 degrees
        new float[]{51.574802f, -126.77165f}, // for 180 degrees
        new float[]{-29.13386f,  -125.98425f}  // for -90/270 degrees
    };
    SensorNum[] sensorPosition = new SensorNum[] // which ultra sonic sensor is in the X direction for each 90 degree increment
    {
        SensorNum.ONE, // for 0 degrees
        SensorNum.TWO, // for 90 degrees
        SensorNum.ONE, // for 180 degrees
        SensorNum.TWO  // for -90/270 degrees
    };
    MathSign[][] operations = new MathSign[][] //whether the distance from the ultrasonic sensor should be added or removed for each 90 degree increment
    {
        new MathSign[]{MathSign.ADD, MathSign.SUBTRACT}, // for 0 degrees
        new MathSign[]{MathSign.SUBTRACT, MathSign.SUBTRACT}, // for 90 degrees
        new MathSign[]{MathSign.SUBTRACT, MathSign.ADD}, // for 180 degrees
        new MathSign[]{MathSign.ADD, MathSign.ADD}  // for -90/270 degrees
    };
    double angleTolerance = 7.5; // how far from each 90 degree increment can the robot be for the ultra sonic to still be valid
    int minMeasureDelay = 0; //how long before the sensors can measure again in ms
    float[] maxDistanceSensorChange = {10,10}; //max distance travalable in one second(in inches)

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