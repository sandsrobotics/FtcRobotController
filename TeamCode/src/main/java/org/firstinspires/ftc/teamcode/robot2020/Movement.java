package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Movement
{
    //////////////////
    //user variables//
    //////////////////
    public static double ticksPerInchForward = 44;
    public static double ticksPerInchSideways = 88;
    public static PIDCoefficients turnPID = new PIDCoefficients(.025,0,0);


    ///////////////////
    //other variables//
    ///////////////////
    protected double speedMultiplier = 1;
    protected double speedMultiplierMin = .2;
    protected double speedMultiplierMax = 2;

    //other class
    Robot robot;

    Movement(Robot robot)
    {
        this.robot = robot;
    }

    ////////////////
    //turn methods//
    ////////////////

    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime)
    {
        double currentAngle = robot.position.currentRotation;
        double error = robot.findAngleError(currentAngle, targetAngle);

        if(Math.abs(error) > tolerance)
        {
            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;
            double totalError = 0;
            double lastError;
            double power;

            while(numberOfTimesInTolerance < numberOfTimesToStayInTolerance)
            {
                lastError = error;
                error = robot.findAngleError(robot.position.currentRotation, targetAngle);
                totalError += error;
                power = getCorrectionFromPID(error, totalError, error - lastError);

                robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(0,0,power));

                if(Math.abs(error) < tolerance) numberOfTimesInTolerance++;
                else numberOfTimesInTolerance = 0;

                numberOfTimesRun++;

                if(numberOfTimesRun > maxRuntime || robot.stop()) break;

                if(robot.debug_methods)
                {
                    if(robot.debug_telemetry)
                    {
                        robot.telemetry.addData("angle error: ", error);
                        robot.telemetry.addData("total angle error: ", totalError);
                        robot.telemetry.addData("angle error change: ", error - lastError);
                        robot.telemetry.addData("current power: ", power);
                        robot.telemetry.addData("number of times run: ", numberOfTimesRun);
                        robot.telemetry.addData("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                    if(robot.debug_dashboard)
                    {
                        robot.packet.put("angle error: ", error);
                        robot.packet.put("total angle error: ", totalError);
                        robot.packet.put("angle error change: ", error - lastError);
                        robot.packet.put("current power: ", power);
                        robot.packet.put("number of times run: ", numberOfTimesRun);
                        robot.packet.put("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                }

                robot.sendTelemetry();
            }

            robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
        }
    }

    //////////////////
    //strafe methods//
    //////////////////
    void strafeSidewaysTicks(int ticks, double power)
    {
        int[] arr = {ticks, -ticks, -ticks, ticks};
        robot.motorConfig.moveMotorForwardSeparateAmountList(robot.motorConfig.driveMotors, arr, power);
        robot.motorConfig.waitForMotorsToFinishList(robot.motorConfig.driveMotors);
    }

    void strafeSidewaysInches(double inches, double power)
    {
        strafeSidewaysTicks((int)(ticksPerInchSideways * inches), power);
    }

    ////////////////
    //move methods//
    ////////////////
    void moveForwardInches(double inches, double power)
    {
        robot.motorConfig.moveMotorsForwardList(robot.motorConfig.driveMotors, (int)(ticksPerInchForward * inches), power);
        robot.motorConfig.waitForMotorsToFinishList(robot.motorConfig.driveMotors);
    }

    void moveAtAngleWithPower(double angle, double power) //in this method angle should be from -180 to 180
    {
        robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveAtAnglePowers(angle,power));
    }

    void moveAtAngleToInches(double angle, double power, double inches)
    {
        double[] arr = moveAtAnglePowers(-angle, power);

        int totalTicks = (int)((inches*ticksPerInchForward*robot.getXYFromAngle(-angle)[1]) + (inches*ticksPerInchSideways*robot.getXYFromAngle(-angle)[0]));

        int i = 0;
        for(DcMotor motor: robot.motorConfig.driveMotors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(totalTicks * arr[i]));
            arr[i] = Math.abs(arr[i]);
            i++;
        }

        robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, arr);
        robot.motorConfig.setMotorsToRunToPositionList(robot.motorConfig.driveMotors);
        robot.motorConfig.waitForMotorsToFinishList(robot.motorConfig.driveMotors);
    }

    //////////
    //teleOp//
    //////////

    void setSpeedMultiplier(double amount)
    {
        if(amount > speedMultiplierMax)
        {
            if (robot.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is greater than max speed. setting to max speed");
            amount = speedMultiplierMax;
        }
        else if(amount < speedMultiplierMin)
        {
            if (robot.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is less than min speed. setting to min speed");
            amount = speedMultiplierMin;
        }
        speedMultiplier = amount;
    }
    void setSpeedMultiplierToMax() { speedMultiplier = speedMultiplierMax; }
    void setSpeedMultiplierToMin() { speedMultiplier = speedMultiplierMin; }

    void moveForTeleOp(Gamepad gamepad1)
    {
        robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
    }

    void headlessMoveForTeleOp( Gamepad gamepad1, double offset)
    {
        double curAngle = -robot.position.currentRotation;
        double gamepadAngle = robot.getAngleFromXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double error = -robot.findAngleError(curAngle,gamepadAngle);
        double power = Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y));
        double[] XY = robot.getXYFromAngle(error);
        XY[0] *= power;
        XY[1] *= power;
        robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(XY[0],XY[1],gamepad1.right_stick_x));
    }

    /////////
    //other//
    /////////

    double[] moveRobotPowers(double X, double Y, double rotation)
    {
        double[] arr =
        {
            (Y + X + rotation),
            (Y - X + rotation),
            (Y - X - rotation),
            (Y + X - rotation)
        };
        double highestPower = 0;

        for(double val:arr) if(val > highestPower) highestPower = val;
        if(highestPower > 1) for(int i = 0; i < 4; i++) arr[i] /= highestPower;
        for(int i = 0; i < 4; i++) arr[i] *= speedMultiplier;
        return (arr);
    }

    double[] moveAtAnglePowers(double angle, double basePower)
    {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0];
        double y = arr[1];

        return moveRobotPowers(x,y,0);
    }

    double getCorrectionFromPID(double error, double totalError, double rateOfChange) // this method takes values from -180 to 180 and returns a value from -1 to 1
    {
        double value;
        value = (error * turnPID.p) + (totalError * turnPID.d) + (rateOfChange * turnPID.i);
        return Math.max(Math.min(value , 1), -1);
    }

    double getCorrectionFromPID(double error, double totalError, double rateOfChange, double P, double I, double D) // this method takes values from -180 to 180 and returns a value from -1 to 1
    {
        double value;
        value = (error * P) + (totalError * I) + (rateOfChange * D);
        return Math.max(Math.min(value , 1), -1);
    }

/*
    void findBlueTowerGoal(double rotationRange, double rotationIncrements)
    {
        turnToAngleSimple(0,5,20,1000);
        if(robot.vision.findTrackable(0,true))
        {
            turnToAngleSimple(robot.vision.lastRotation.thirdAngle, 1, 25,2000);
        }
        else
        {
            robot.movement.turnToAngleSimple(-rotationRange, 5,20,1000);
            robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(0,0,.1));
            while(robot.getAngles().thirdAngle < rotationRange)
            {
                //robot.motorConfig.setMotorsToSeparatePowersArray(moveRobotPowers(0,0,.15));
                if(robot.vision.findTrackable(0,true))
                {
                    turnToAngleSimple(robot.vision.lastRotation.thirdAngle , 1, 25,2000);
                    break;
                }
                if(robot.stop()) break;
            }
            robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
        }
    }
 */
}
