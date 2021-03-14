package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test power shots")
public class Test3 extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.usePositionThread = true;
        ru.usePositionTracking = true;
        ru.useDistanceSensors = true;
        ru.useDrive = true;
        ru.useLauncher = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(false);

        robot.launcher.autoLaunchPowerShotsV2();
    }
}