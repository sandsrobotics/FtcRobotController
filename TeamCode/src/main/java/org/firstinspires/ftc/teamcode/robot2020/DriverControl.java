package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "driver control")
public class DriverControl extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.useVuforia = false;
        ru.useComplexMovement = false;

        robot = new Robot(this,ru);

        waitForStart();

        robot.start();


        while (opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1, GamepadButtons.leftBUMPER);
            robot.grabber.runForTeleop(gamepad1);
            robot.launcher.opModeRun(gamepad2,false);
        }
    }
}
