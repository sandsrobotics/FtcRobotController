package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Movement
{

    ///////////////////
    //other variables//
    ///////////////////
    //for move robot
    protected double[] lastMovePowers = new double[]{0,0,0};

    //other class
    Robot robot;
    MovementSettings movementSettings;

    Movement(Robot robot)
    {
        movementSettings = new MovementSettings();
        this.robot = robot;
    }
    Movement(Robot robot, MovementSettings movementSettings)
    {
        this.movementSettings = movementSettings;
        this.robot = robot;
    }

    ////////////////
    //turn methods//
    ////////////////

    void turnToAngle(double targetAngle, double tolerance, int numberOfTimesToStayInTolerance, int maxRuntime, double maxSpeed)
    {
        double error = robot.findAngleError(robot.position.currentRotation, targetAngle);

        if(Math.abs(error) > tolerance) {

            int numberOfTimesInTolerance = 0;
            PID pid = new PID(movementSettings.turnPID, -maxSpeed, maxSpeed);

            while (numberOfTimesInTolerance < numberOfTimesToStayInTolerance && maxRuntime > 0 && !robot.stop())
            {
                error = robot.findAngleError(robot.position.currentRotation, targetAngle);
                pid.updatePID(error);

                robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(0, 0, pid.returnValue(),false,true));

                if (Math.abs(error) < tolerance) numberOfTimesInTolerance++;
                else numberOfTimesInTolerance = 0;

                maxRuntime--;
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
        strafeSidewaysTicks((int)(movementSettings.ticksPerInchSideways * inches), power);
    }

    ////////////////
    //move methods//
    ////////////////
    void moveForwardInches(double inches, double power)
    {
        robot.motorConfig.moveMotorsForwardList(robot.motorConfig.driveMotors, (int)(movementSettings.ticksPerInchForward * inches), power);
        robot.motorConfig.waitForMotorsToFinishList(robot.motorConfig.driveMotors);
    }

    void moveToPosition(double[] targetPos, double[] tol, int timesToStayInTolerance, int maxLoops, PIDCoefficients moveXPID, PIDCoefficients moveYPID, PIDCoefficients turnPID, double maxSpeed)
    {
        if(robot.robotUsage.usePositionTracking)
        {
            double[] currentPos = robot.position.currentRobotPosition;

            if (Math.abs(targetPos[0] - currentPos[0]) > tol[0] || Math.abs(targetPos[1] - currentPos[1]) > tol[1] || Math.abs(targetPos[2] - currentPos[2]) > tol[2]) {
                PID xPID = new PID(moveXPID, -maxSpeed, maxSpeed);
                PID yPID = new PID(moveYPID, -maxSpeed, maxSpeed);
                PID rotPID = new PID(turnPID, -maxSpeed, maxSpeed);

                double[] powers = new double[3];

                double errorVectorRot;
                double errorVectorMag;

                int numOfTimesInTolerance = 0;

                while (!robot.stop() && (maxLoops > 0) && (numOfTimesInTolerance < timesToStayInTolerance)) {
                    currentPos = robot.position.currentRobotPosition;

                    //calculate the error vector
                    errorVectorMag = Math.sqrt(Math.pow((targetPos[0] - currentPos[0]), 2) + Math.pow((targetPos[1] - currentPos[1]), 2));
                    errorVectorRot = Math.toDegrees(Math.atan2((targetPos[0] - currentPos[0]), (targetPos[1] - currentPos[1])));

                    //take out robot rotation
                    errorVectorRot -= currentPos[2];
                    errorVectorRot = robot.scaleAngle(errorVectorRot);

                    //get the errors comps
                    powers[0] = xPID.updatePIDAndReturnValue(errorVectorMag * Math.sin(Math.toRadians(errorVectorRot)));
                    powers[1] = yPID.updatePIDAndReturnValue(errorVectorMag * Math.cos(Math.toRadians(errorVectorRot)));
                    powers[2] = rotPID.updatePIDAndReturnValue(robot.findAngleError(currentPos[2], targetPos[2]));

                    if (Math.abs(targetPos[0] - currentPos[0]) < tol[0] && Math.abs(targetPos[1] - currentPos[1]) < tol[1] && Math.abs(targetPos[2] - currentPos[2]) < tol[2])
                        numOfTimesInTolerance++;
                    else numOfTimesInTolerance = 0;

                    robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(powers[0], powers[1], powers[2], false, true));
                    maxLoops--;
                    if(robot.robotSettings.debug_methods)
                    {
                        robot.addTelemetry("x: ", robot.position.currentRobotPosition[0]);
                        robot.addTelemetry("y: ", robot.position.currentRobotPosition[1]);
                        robot.addTelemetry("rot: ", robot.position.currentRobotPosition[2]);
                        robot.addTelemetry("error mag: ", errorVectorMag);
                        robot.addTelemetry("error rot: ", errorVectorRot);
                        robot.sendTelemetry();
                    }
                }
            }
            robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
        }
        else if(robot.robotSettings.debug_methods) robot.addTelemetry("error in Movement.moveToPosition: ", "robot can not move to position because it does not know its position");
    }

    void moveToPosition(double[] targetPos, double[] tol, int timesToStayInTolerance, int maxLoops, double maxSpeed) { moveToPosition(targetPos, tol, timesToStayInTolerance, maxLoops, movementSettings.moveXPID, movementSettings.moveYPID, movementSettings.turnPID, maxSpeed); }

    //////////
    //teleOp//
    //////////

    void setSpeedMultiplier(double amount)
    {
        if(amount > movementSettings.speedMultiplierMax)
        {
            if (robot.robotSettings.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is greater than max speed. setting to max speed");
            amount = movementSettings.speedMultiplierMax;
        }
        else if(amount < movementSettings.speedMultiplierMin)
        {
            if (robot.robotSettings.debug_methods) robot.addTelemetry("warning in Movement.setSpeedMultiplier: ", "set speed is less than min speed. setting to min speed");
            amount = movementSettings.speedMultiplierMin;
        }
        movementSettings.speedMultiplier = amount;
    }
    void setSpeedMultiplierToMax() { movementSettings.speedMultiplier = movementSettings.speedMultiplierMax; }
    void setSpeedMultiplierToMin() { movementSettings.speedMultiplier = movementSettings.speedMultiplierMin; }

    void moveForTeleOp(Gamepad gamepad1, GamepadButtons breakButton, boolean useTelemetry)
    {
        if(breakButton.getButtonHeld(gamepad1))
        {
            robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
            lastMovePowers[0] = 0; lastMovePowers[1] = 0; lastMovePowers[2] = 0;
        }
        else robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true,true));
        if(useTelemetry) teleOpTelemetry();
    }

    void teleOpTelemetry()
    {

    }

    void headlessMoveForTeleOp(Gamepad gamepad1, double offset)
    {
        double curAngle = -robot.position.currentRotation + offset;
        curAngle = robot.scaleAngle(curAngle);
        double gamepadAngle = robot.getAngleFromXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double error = -robot.findAngleError(curAngle,gamepadAngle);
        double power = Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y));
        double[] XY = robot.getXYFromAngle(error);
        XY[0] *= power;
        XY[1] *= power;
        robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(XY[0],XY[1],gamepad1.right_stick_x, true, true));
    }

    /////////
    //other//
    /////////

    double[] moveRobotPowers(double X, double Y, double rotation, boolean applySpeedMultiplier, boolean applyMoveSmoothing)
    {
        if(applyMoveSmoothing)
        {
            //smoothing for XYR
            X = applySmoothing(X, lastMovePowers[0], movementSettings.moveXSmoothingSteps);
            Y = applySmoothing(Y, lastMovePowers[1], movementSettings.moveYSmoothingSteps);
            rotation = applySmoothing(rotation, lastMovePowers[2], movementSettings.rotationSmoothingSteps);

            lastMovePowers[0] = X;
            lastMovePowers[1] = Y;
            lastMovePowers[2] = rotation;
        }

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
        if(applySpeedMultiplier)for(int i = 0; i < 4; i++) arr[i] *= movementSettings.speedMultiplier;
        return (arr);
    }

    double[] moveAtAnglePowers(double angle, double basePower, boolean applySmoothing)
    {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0] * basePower;
        double y = arr[1] * basePower;

        return moveRobotPowers(x,y,0, false,applySmoothing);
    }

    double applySmoothing(double currentVal, double lastVal, double smoothingSteps)
    {
        if(currentVal - lastVal > smoothingSteps) { currentVal = lastVal + smoothingSteps; }
        else if(currentVal - lastVal < -smoothingSteps) { currentVal = lastVal - smoothingSteps; }
        return currentVal;
    }
}

class MovementSettings
{
    //////////////////
    //user variables//
    //////////////////
    public static double ticksPerInchForward = 44;
    public static double ticksPerInchSideways = 51.3;
    public static PIDCoefficients turnPID = new PIDCoefficients(.04,0,0);
    public static PIDCoefficients moveXPID = new PIDCoefficients(.06,0,0);
    public static PIDCoefficients moveYPID = new PIDCoefficients(.06,0,0);
    public static double moveXSmoothingSteps = .1;
    public static double moveYSmoothingSteps = .1;
    public static double rotationSmoothingSteps = .1;

    protected double speedMultiplier = 1;
    protected final double speedMultiplierMin = .2;
    protected final double speedMultiplierMax = 2;

    MovementSettings(){}
}