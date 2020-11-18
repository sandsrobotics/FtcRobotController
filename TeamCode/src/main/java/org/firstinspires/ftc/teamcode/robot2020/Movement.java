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
    public static PIDCoefficients movePID = new PIDCoefficients(.05,0,0);

    protected double speedMultiplier = 1;
    protected final double speedMultiplierMin = .2;
    protected final double speedMultiplierMax = 2;

    ///////////////////
    //other variables//
    ///////////////////
    //for move robot
    protected double[] lastMovePowers = new double[]{0,0,0};

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
        double error = robot.findAngleError(robot.position.currentRotation, targetAngle);

        if(Math.abs(error) > tolerance) {

            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;
            double power;
            PID pid = new PID(turnPID, -1, 1);

            while (numberOfTimesInTolerance < numberOfTimesToStayInTolerance && numberOfTimesRun < maxRuntime && !robot.stop())
            {
                error = robot.findAngleError(robot.position.currentRotation, targetAngle);
                power = pid.updatePIDAndReturnValue(error);

                robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(0, 0, power));

                if (Math.abs(error) < tolerance) numberOfTimesInTolerance++;
                else numberOfTimesInTolerance = 0;

                numberOfTimesRun++;

                robot.addTelemetry("angle error: ", error);
                robot.addTelemetry("total angle error: ", pid.totalError);
                robot.addTelemetry("angle error change: ", pid.currentError - pid.lastError);
                robot.addTelemetry("current power: ", power);
                robot.addTelemetry("number of times run: ", numberOfTimesRun);
                robot.addTelemetry("number of times in tolerance: ", numberOfTimesInTolerance);
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

    void moveToPosition(double[] targetPos, double[] tol, int maxLoops, int timesToStayInTolerance, PIDCoefficients movePID, PIDCoefficients turnPID, double maxSpeed)
    {
        PID xPID = new PID(movePID, -maxSpeed, maxSpeed);
        PID yPID = new PID(movePID, -maxSpeed, maxSpeed);
        PID rotPID = new PID(turnPID, -maxSpeed, maxSpeed);

        double[] currentPos;
        double[] powers = new double[3];

        double errorVectorRot;
        double errorVectorMag;

        int numOfTimesInTolerance = 0;

        while (!robot.stop() && maxLoops > 0 && numOfTimesInTolerance < timesToStayInTolerance)
        {
            currentPos = robot.position.currentRobotPosition;

            //calculate the error vector
            errorVectorMag = Math.sqrt(Math.pow((targetPos[0] - currentPos[0]), 2) + Math.pow((targetPos[1] - currentPos[1]), 2));
            errorVectorRot = Math.atan2((targetPos[1] - currentPos[1]),(targetPos[0] - currentPos[0]));

            //take out robot rotation
            errorVectorRot -= currentPos[2];
            errorVectorRot = robot.scaleAngle(errorVectorRot);

            //get the errors comps
            powers[0] = xPID.updatePIDAndReturnValue(errorVectorMag * Math.sin(errorVectorRot));
            powers[1] = yPID.updatePIDAndReturnValue(errorVectorMag * Math.cos(errorVectorRot));
            powers[2] = rotPID.updatePIDAndReturnValue(robot.findAngleError(currentPos[2], targetPos[2]));

            if(Math.abs(targetPos[0] - currentPos[0]) < tol[0] && Math.abs(targetPos[1] - currentPos[1]) < tol[1] && Math.abs(targetPos[2] - currentPos[2]) < tol[2])
            {
                numOfTimesInTolerance++;
            }

            robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(powers[0], powers[1], powers[2]));
            maxLoops--;
        }
        robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
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

    void moveForTeleOp(Gamepad gamepad1 , GamepadButtons breakButton)
    {
        if(breakButton.getButtonPressed(gamepad1))
        {
            robot.motorConfig.stopMotorsList(robot.motorConfig.driveMotors);
            lastMovePowers[0] = 0; lastMovePowers[1] = 0; lastMovePowers[2] = 0;

        }
        else robot.motorConfig.setMotorsToSeparatePowersArrayList(robot.motorConfig.driveMotors, moveRobotPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, .05 , .05));
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

    double[] moveRobotPowers(double X, double Y, double rotation, double moveSmoothingSteps, double rotationSmoothingSteps)
    {
        //smoothing for XYR
        X = applySmoothing(X, lastMovePowers[0], moveSmoothingSteps);
        Y = applySmoothing(Y, lastMovePowers[1], moveSmoothingSteps);
        rotation = applySmoothing(rotation, lastMovePowers[2], rotationSmoothingSteps);

        lastMovePowers[0] = X;
        lastMovePowers[1] = Y;
        lastMovePowers[2] = rotation;

        return moveRobotPowers(X, Y, rotation);
    }

    double[] moveAtAnglePowers(double angle, double basePower)
    {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0] * basePower;
        double y = arr[1] * basePower;

        return moveRobotPowers(x,y,0);
    }

    double[] moveAtAnglePowers(double angle, double basePower, double moveSmoothingSteps, double rotationSmoothingSteps)
    {
        double[] arr;
        arr = robot.getXYFromAngle(angle);
        double x = arr[0] * basePower;
        double y = arr[1] * basePower;

        return moveRobotPowers(x,y,0, moveSmoothingSteps, rotationSmoothingSteps);
    }

    double applySmoothing(double currentVal, double lastVal, double smoothingSteps)
    {
        if(currentVal - lastVal > smoothingSteps) { currentVal = lastVal + smoothingSteps; }
        else if(currentVal - lastVal < -smoothingSteps) { currentVal = lastVal - smoothingSteps; }
        return currentVal;
    }
}
