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

        //with encoders
        double ticksPerRotation = (2 * wheelDistanceFromCenter * Math.PI) * ((Movement.ticksPerInchForward + Movement.ticksPerInchSideways) / 2);
        List<double[]> recordedMotorsAndAngles = new ArrayList<>();
        double[] lastMotorsAndAngles;

        //with accelerometer
        double[] recordedAccelAndTime;
        double[] lastRecodedAccelAndTime;

    //rotation
    volatile Orientation currentAllAxisRotations;
    volatile double currentRotation;
    protected double rotationOffset = -startRotation;

    //velocity
    volatile Velocity currentVelocity;

    //angular velocity
    volatile AngularVelocity currentAngularVelocity;

    //acceleration
    volatile Acceleration currentAcceleration;

    //other class
    Robot robot;

    Position(Robot robot){this.robot = robot;}

    Orientation updateAngles()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angles.thirdAngle *= -1;
        angles.thirdAngle -= rotationOffset;
        if(angles.thirdAngle < -180) {angles.thirdAngle = 360 + angles.thirdAngle;}
        else if(angles.thirdAngle > 180){angles.thirdAngle = angles.thirdAngle - 360;}
        return angles;
    }

    void resetZRotationAxis() {rotationOffset = -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle;}

    ////////////////////
    //position finding//
    ////////////////////
    void updatePosWithEncodersAndAngles()
    {
        addEncoderAndAngleDataPoint();

        double[] totalMovement = new double[]{0,0,0};
        double[] movementPerDataPoint;

        robot.addDoubleArrays(totalMovement, getMovementFromMotorAndAngle(recordedMotorsAndAngles.get(0), lastMotorsAndAngles));

        for(int i = 1; i < recordedMotorsAndAngles.size(); i++)
        {
            robot.addDoubleArrays(totalMovement, getMovementFromMotorAndAngle(recordedMotorsAndAngles.get(0), lastMotorsAndAngles));
        }

        addEncoderAndAngleDataPoint();
        lastMotorsAndAngles = recordedMotorsAndAngles.get(-1);

        recordedMotorsAndAngles.clear();

        robot.addDoubleArrays(currentPosition, totalMovement);
    }



    double[] getMovementFromMotorAndAngle(double[] curMotorPosAndAngles, double[] lastMotorPosAndAngles)
    {
        double[] moved = new double[3];

        double[] motorDifference = new double[curMotorPosAndAngles.length - 1];
        for(int i = 0; i < motorDifference.length; i++) motorDifference[i] = (curMotorPosAndAngles[i + 1] - lastMotorPosAndAngles[i + 1]);

        //set and take rotation out of motor movement
        moved[2] = robot.findAngleError(curMotorPosAndAngles[0], lastMotorPosAndAngles[0]);
        int ticksRotated = (int)((moved[2] / 360) * ticksPerRotation);
        motorDifference[0] -= ticksRotated;
        motorDifference[1] -= ticksRotated;
        motorDifference[2] += ticksRotated;
        motorDifference[3] += ticksRotated;



        return moved;
    }

    void updateMovementFromAccelerometer()
    {
        double[] moved = new double[3];

        currentPosition[2] = currentRotation;


    }

    void addEncoderAndAngleDataPoint()
    {
        int[] positions = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
        double[] out = new double[positions.length + 1];
        for(int i = 1; i < out.length; i++) out[i] = positions[i-1];
        out[0] = currentRotation;
        recordedMotorsAndAngles.add(out);
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
    @Override
    public void run()
    {
        while (!Thread.currentThread().isInterrupted() && robot.opMode.opModeIsActive())
        {
            //put run stuff in here
            currentAcceleration = robot.imu.getAcceleration();
            currentVelocity = robot.imu.getVelocity();
            currentAngularVelocity = robot.imu.getAngularVelocity();
            currentAllAxisRotations = updateAngles();
            currentRotation = currentAllAxisRotations.thirdAngle;
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
