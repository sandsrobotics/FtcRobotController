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

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2);

        robot.vision.initVuforia();
        robot.vision.startDashboardCameraStream(24);
        robot.vision.loadAsset("UltimateGoal");
        robot.vision.setAllTrackablesNames();
        robot.vision.setAllTrackablesPosition();
        robot.vision.setPhoneTransform(new float[]{0,0,0}, new float[]{0,0,0});

        waitForStart();

        robot.startTelemetry();
        robot.vision.activate();

        while (opModeIsActive())
        {
            robot.telemetry.addData("",robot.vision.findAnyTrackable());
            robot.movement.moveForTeleOp(gamepad1);
            robot.sendTelemetry();
        }

        robot.vision.deactivate();
    }
}
