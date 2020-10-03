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
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2, true, false, true);

        if(robot.debug_dashboard)robot.vision.startDashboardCameraStream(24);

        waitForStart();

        robot.startTelemetry();
        robot.vision.activate();

        while (opModeIsActive())
        {
            robot.vision.findAnyTrackable(true);
            robot.vision.printTelemetry();
            robot.movement.moveForTeleOp(gamepad1);
            robot.sendTelemetry();
            if(gamepad1.a) robot.movement.findBlueTowerGoal(45,22.5);
        }

        robot.vision.deactivate();
        if(robot.debug_dashboard)robot.vision.stopDashboardCameraStream();
    }
}
