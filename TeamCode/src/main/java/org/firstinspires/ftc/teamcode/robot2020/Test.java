package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test position tracking")
public class Test extends LinearOpMode
{

    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(this,true, false, false, false, false);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.position.start();

        while(opModeIsActive())
        {
            robot.startTelemetry();

            robot.addTelemetry("velocity: ", robot.position.currentVelocity);

            robot.movement.moveForTeleOp(gamepad1);

            robot.sendTelemetry();
        }
    }
}
