package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test launcher v1.1")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useLauncher = true;
        ru.useDrive = true;
        robot = new Robot(this, ru);

        waitForStart();

        robot.start();

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.dpadUP,true);
            robot.launcher.runForTeleOp(gamepad1, true);
            robot.sendTelemetry();
        }

    }
}
