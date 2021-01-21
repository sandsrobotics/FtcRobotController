package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// test
@Config
@TeleOp(name = "test Tfod")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useTensorFlow = true;
        ru.useVuforia = true;
        ru.useTensorFlowInTread = false;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        robot.vision.activateTfod();
        robot.vision.tfod.setZoom(1.5, 16/9);
        FtcDashboard.getInstance().startCameraStream(robot.vision.tfod, 0);

        waitForStart();

        robot.start();

        while(opModeIsActive())
        {
            robot.vision.findAllTfodObjects();
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.dpadUP,true);
            if(robot.vision.anyTfodObjectsFound)
            {
                telemetry.addData("# Object Detected", robot.vision.tfodCurrentRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : robot.vision.tfodCurrentRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }
            robot.sendTelemetry();
        }

    }
}
