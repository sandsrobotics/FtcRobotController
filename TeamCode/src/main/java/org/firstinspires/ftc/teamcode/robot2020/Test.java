package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test vision v1.1")
public class Test extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,true, true, false, false);

        robot.startTelemetry();
        robot.launcher.readCsv();
        robot.sendTelemetry();

        waitForStart();

        while (opModeIsActive())
        {
            robot.startTelemetry();
            robot.packet.put("check",robot.launcher.calibrationValues);
            robot.sendTelemetry();
        }
    }
}
