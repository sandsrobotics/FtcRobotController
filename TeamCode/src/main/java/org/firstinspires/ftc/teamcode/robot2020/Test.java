package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test position tracking v2.4")
public class Test extends LinearOpMode
{
    public static int calTime = 1;
    Velocity diff;

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,true, false, false, false, false);
        //robot.movement.setSpeedMultiplier(.25);

        waitForStart();

        robot.startTelemetry();
        robot.position.start();

        while (opModeIsActive())
        {
            robot.movement.headlessMoveForTeleOp(gamepad1 , 0);
            robot.addTelemetry("position x: ", robot.position.currentRobotPosition[0]);
            robot.addTelemetry("position y: ", robot.position.currentRobotPosition[1]);
            robot.addTelemetry("rotation: ", robot.position.currentRobotPosition[2]);
            robot.sendTelemetry();
        }
    }
}
