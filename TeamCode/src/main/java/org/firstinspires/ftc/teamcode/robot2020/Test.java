package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// test
@Config
@TeleOp(name = "test move functions v1.2.7")
public class Test extends LinearOpMode
{
    Robot robot;

    public static double power = .5;
    public static double targetAngle = 45;
    public static double distanceForward = 12;
    public static double distanceSideways = 12;
    public static double moveAngle = 45;
    public static double moveDistance = 12;
    private boolean runningHeadless = false;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.motorConfig.resetEncoders();
        robot.motorConfig.setMotorsToRunWithEncoders();

        robot.motorConfig.setMotorsToBrake();

        robot.debug_methods = true;
        robot.debug_imu = false;
        robot.debug_motors = false;



        waitForStart();

        robot.startTelemetry();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                robot.movement.turnToAngleSimple(targetAngle,.5,50,1000);
            }
            if(gamepad1.b)
            {
                robot.movement.moveForwardInches(distanceForward,power);
                robot.motorConfig.setMotorsToRunWithEncoders();
            }
            if(gamepad1.x)
            {
                robot.resetZaxis();
                //robot.movement.strafeSidewaysInches(distanceSideways,power);
                //robot.motorConfig.setMotorsToRunWithEncoders();
            }
            if(gamepad1.y)
            {
                runningHeadless = !runningHeadless;
            }
            if(!runningHeadless)
            {
                robot.movement.moveForTeleOp(gamepad1);
                robot.addTelemetryString("headless mode: ", "disabled");
            }
            else
            {
                robot.movement.headlessMoveForTeleOp(gamepad1,0);
                robot.addTelemetryString("headless mode: ", "enabled");
            }

            robot.addTelemetryDouble("angle: ", robot.getAngles().thirdAngle);
            robot.sendTelemetry();
        }
    }
}
