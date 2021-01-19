package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start();

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.dpadUP,true);
            if(robot.vision.anyTfodObjectsFound)
            {
                telemetry.addData("# Object Detected", robot.vision.tfodUpdatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : robot.vision.tfodUpdatedRecognitions) {
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
