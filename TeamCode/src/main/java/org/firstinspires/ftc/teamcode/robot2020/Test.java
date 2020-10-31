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

    int[] pos = new int[4];

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,true, false, false, false, false);

        robot.movement.setSpeedMultiplier(.5);

        waitForStart();

        while (opModeIsActive())
        {
            robot.startTelemetry();

            robot.movement.moveForTeleOp(gamepad1);
            pos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
            for(int i = 0; i < 4; i++) robot.addTelemetryDouble("motor " + i + ": ", pos[i]);
            robot.sendTelemetry();
        }
    }
}
