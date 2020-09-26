package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Movement
{
    //////////////////
    //user variables//
    //////////////////
    public static double ticksPerInchForward = 44;
    public static double ticksPerInchSideways = 88;
    public static PIDCoefficients turnPID = new PIDCoefficients(.025,0,0);

    //other class
    Robot robot;

    Movement(Robot robot)
    {
        this.robot = robot;
    }

    ////////////////
    //turn methods//
    ////////////////
    void turnToAngPID(double targetAngle, double tolerance, int numOfTimesToStayInTolerance, int maxRuntime)
    {
        double I = 0;
        double currentAngle = robot.getAngles().thirdAngle;
        double error = robot.findAngleError(currentAngle, targetAngle);
        double lastError;
        double pow;
        int numOfTimesInTolerance = 0;
        int numOfTimesRun = 0;

        while(numOfTimesInTolerance < numOfTimesToStayInTolerance)
        {
            lastError = error;
            currentAngle = robot.getAngles().thirdAngle;
            error = robot.findAngleError(currentAngle, targetAngle);
            pow = robot.getCorrectionFromPID(error, lastError,0,.1);

            if(Math.abs(error) < tolerance)
            {
                numOfTimesInTolerance ++;
                //I = 0;
                pow = 0;
            }
            else numOfTimesInTolerance = 0;

            robot.motorConfig.setMotorsToSeparatePowersArray(moveRobotPowers(0,0,pow));

            numOfTimesRun ++;
            if(numOfTimesRun > maxRuntime || Robot.emergencyStop || robot.gamepad1.back || robot.gamepad2.back) break;
        }
        robot.motorConfig.stopMotors();
    }

    void turnToAngleSimple(double targetAngle, double tolerance, double numberOfTimesToStayInTolerance, double maxRuntime)
    {
        double currentAngle = robot.getAngles().thirdAngle;
        double error = robot.findAngleError(currentAngle, targetAngle);

        if(Math.abs(error) > tolerance)
        {
            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;

            //I = 0;
            //robot.motorConfig.setMotorsToRunWithEncoders();
            //robot.motorConfig.setMotorsToBrake();

            while(numberOfTimesInTolerance < numberOfTimesToStayInTolerance)
            {
                currentAngle = robot.getAngles().thirdAngle;
                error = robot.findAngleError(currentAngle, targetAngle);
                robot.motorConfig.setMotorsToSeparatePowersArray(moveRobotPowers(0,0,error * turnPID.p));

                if(Math.abs(error) < tolerance)
                {
                    robot.I = 0;
                    numberOfTimesInTolerance++;
                }
                else {
                    numberOfTimesInTolerance = 0;
                }
                numberOfTimesRun++;

                if(Robot.emergencyStop || numberOfTimesRun > maxRuntime || robot.gamepad1.back || robot.gamepad2.back) break;

                if(robot.debug_methods)
                {
                    if(robot.debug_telemetry)
                    {
                        robot.telemetry.addData("angle error: ", error);
                        robot.telemetry.addData("current power: ", error * turnPID.p);
                        robot.telemetry.addData("number of times run: ", numberOfTimesRun);
                        robot.telemetry.addData("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                    if(robot.debug_dashboard)
                    {
                        robot.packet.put("angle error: ", error);
                        robot.packet.put("current power: ", error * turnPID.p);
                        robot.packet.put("number of times run: ", numberOfTimesRun);
                        robot.packet.put("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                }
                robot.updateTelemetry();
                robot.sendTelemetry();
            }
            robot.motorConfig.stopMotors();
        }
    }

    //////////////////
    //strafe methods//
    //////////////////
    void strafeSidewaysTicks(int ticks, double power)
    {
        int[] arr = {ticks, -ticks, -ticks, ticks};
        robot.motorConfig.moveMotorForwardSeparateAmount(arr,power);
        robot.motorConfig.waitForMotorsToFinish();
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
        robot.motorConfig.moveMotorsForward((int)(ticksPerInchForward * inches), power);
        robot.motorConfig.waitForMotorsToFinish();
    }

    void moveAtAngleWithPower(double angle, double power) //in this method angle should be from -180 to 180
    {
        robot.motorConfig.setMotorsToSeparatePowersArray(moveAtAnglePowers(angle,power));
    }

    void moveAtAngleToInches(double angle, double power, double inches)
    {
        double[] arr = moveAtAnglePowers(-angle, power);

        int totalTicks = (int)((inches*ticksPerInchForward*robot.getXYFromAngle(-angle)[1]) + (inches*ticksPerInchSideways*robot.getXYFromAngle(-angle)[0]));

        int i = 0;
        for(DcMotor motor: robot.motorConfig.motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(totalTicks * arr[i]));
            arr[i] = Math.abs(arr[i]);
            i++;
        }

        robot.motorConfig.setMotorsToSeparatePowersArray(arr);
        robot.motorConfig.setMotorsToRunToPosition();
        robot.motorConfig.waitForMotorsToFinish();
    }

    //////////
    //teleOp//
    //////////
    void moveForTeleOp(Gamepad gamepad1)
    {
        robot.motorConfig.setMotorsToSeparatePowersArray(moveRobotPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
    }

    void headlessMoveForTeleOp( Gamepad gamepad1, double offset)
    {
        double curAngle = -robot.getAngles().thirdAngle;
        double gamepadAngle = robot.getAngleFromXY(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double error = -robot.findAngleError(curAngle,gamepadAngle);
        double power = Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y));
        double[] XY = robot.getXYFromAngle(error);
        XY[0] *= power;
        XY[1] *= power;
        robot.motorConfig.setMotorsToSeparatePowersArray(moveRobotPowers(XY[0],XY[1],gamepad1.right_stick_x));
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
}
